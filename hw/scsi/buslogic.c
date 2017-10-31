#include "qemu/osdep.h"

#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "hw/scsi/scsi.h"
#include "sysemu/dma.h"
#include "qemu/log.h"

#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "hw/scsi/scsi.h"
#include "sysemu/dma.h"
#include "qemu/timer.h"
#include "hw/pci/msi.h"


#define TYPE_BUSLOGIC "BusLogic"
#define PCI_VENDOR_ID_BUSLOGIC 0x104b
#define PCI_DEVICE_ID_BUSLOGIC_BT958 0x1040

#define BUSLOGIC_MAX_DEVICES 16

#define BUSLOGIC_BT958(obj) \
    OBJECT_CHECK(BuslogicState, (obj), TYPE_BUSLOGIC)

#define BL_CTRL_WREG 0        /*write only*/
#define BL_STAT_RREG 0        /*read only*/
#define BL_CMD_WREG 1        /*write only*/
#define BL_DAT_IN_RREG 1        /*read only*/
#define BL_INT_RREG 2        /*read only*/
#define BL_GEO_WREG 3        /*undocumented*/
#define BL_GEO_RREG 3        /*undocumented*/




/* Status codes */
#define SCSI_STATUS_OK                          0x00
#define SCSI_STATUS_CHECK_CONDITION             0x02
#define SCSI_STATUS_CONDITION_MET               0x04
#define SCSI_STATUS_BUSY                        0x08
#define SCSI_STATUS_INTERMEDIATE                0x10
#define SCSI_STATUS_DATA_UNDEROVER_RUN          0x12
#define SCSI_STATUS_INTERMEDIATE_CONDITION_MET  0x14
#define SCSI_STATUS_RESERVATION_CONFLICT        0x18
#define SCSI_STATUS_COMMAND_TERMINATED          0x22
#define SCSI_STATUS_QUEUE_FULL                  0x28
#define SCSI_STATUS_ACA_ACTIVE                  0x30
#define SCSI_STATUS_TASK_ABORTED                0x40





/*control register masks*/
#define BL_CTRL_RSBUS  0x10   /* Reset SCSI Bus. */
#define BL_CTRL_RINT   0x20   /* Reset Interrupt. */
#define BL_CTRL_RSOFT  0x40   /* Soft Reset. */
#define BL_CTRL_RHARD  0x80   /* Hard Reset. */


/*status register masks*/
#define BL_STAT_CMDINV 0x01        /* Command Invalid. */
#define BL_STAT_DIRRDY 0x04   /* Data In Register Ready. */
#define BL_STAT_CPRBSY 0x08   /* Command/Parameter Out Register Busy. */
#define BL_STAT_HARDY  0x10   /* Host Adapter Ready. */
#define BL_STAT_INREQ  0x20   /* Initialization Required. */
#define BL_STAT_DFAIL  0x40   /* Diagnostic Failure. */
#define BL_STAT_DACT   0x80   /* Diagnistic Active. */


/*interrupt register masks*/
#define BL_INTR_IMBL   0x01   /* Incoming Mailbox Loaded. */
#define BL_INTR_OMBR   0x02   /* Outgoing Mailbox Available. */
#define BL_INTR_CMDC   0x04   /* Command Complete. */
#define BL_INTR_RSTS   0x08   /* SCSO Bus Reset State. */
#define BL_INTR_INTV   0x80   /* Interrupt Valid. */

#define BL_GEOM_XLATEN 0x80


/**
 * Commands the BusLogic adapter supports.
 */
#define BL_TEST_CMDC_INTERRUPT  0x00
#define BL_INITIALIZE_MAILBOX  0x01
#define BL_EXECUTE_MAILBOX_COMMAND  0x02
#define BL_EXECUTE_BIOS_COMMAND  0x03
#define BL_INQUIRE_BOARD_ID  0x04
#define BL_ENABLE_OUTGOING_MAILBOX_AVAILABLE_INTERRUPT  0x05
#define BL_SET_SCSI_SELECTION_TIMEOUT  0x06
#define BL_SET_PREEMPT_TIME_ON_BUS  0x07
#define BL_SET_TIME_OFF_BUS  0x08
#define BL_SET_BUS_TRANSFER_RATE  0x09
#define BL_INQUIRE_INSTALLED_DEVICES_ID_0_TO_7  0x0a
#define BL_INQUIRE_CONFIGURATION  0x0b
#define BL_ENABLE_TARGET_MODE  0x0c
#define BL_INQUIRE_SETUP_INFORMATION  0x0d
#define BL_WRITE_ADAPTER_LOCAL_RAM  0x1a
#define BL_READ_ADAPTER_LOCAL_RAM  0x1b
#define BL_WRITE_BUSMASTER_CHIP_FIFO  0x1c
#define BL_READ_BUSMASTER_CHIP_FIFO  0x1d
#define BL_ECHO_COMMAND_DATA  0x1f
#define BL_HOST_ADAPTER_DIAGNOSTIC  0x20
#define BL_SET_ADAPTER_OPTIONS  0x21
#define BL_INQUIRE_INSTALLED_DEVICES_ID_8_TO_15  0x23
#define BL_INQUIRE_TARGET_DEVICES  0x24
#define BL_DISABLE_HOST_ADAPTER_INTERRUPT  0x25
#define BL_EXT_BIOS_INFO  0x28
#define BL_UNLOCK_MAILBOX  0x29
#define BL_INITIALIZE_EXTENDED_MAILBOX  0x81
#define BL_EXECUTE_SCSI_COMMAND  0x83
#define BL_INQUIRE_FIRMWARE_VERSION_3RD_LETTER  0x84
#define BL_INQUIRE_FIRMWARE_VERSION_LETTER  0x85
#define BL_INQUIRE_PCI_HOST_ADAPTER_INFORMATION  0x86
#define BL_INQUIRE_HOST_ADAPTER_MODEL_NUMBER  0x8b
#define BL_INQUIRE_SYNCHRONOUS_PERIOD  0x8c
#define BL_INQUIRE_EXTENDED_SETUP_INFORMATION  0x8d
#define BL_ENABLE_STRICT_ROUND_ROBIN_MODE  0x8f
#define BL_STORE_HOST_ADAPTER_LOCAL_RAM  0x90
#define BL_FETCH_HOST_ADAPTER_LOCAL_RAM  0x91
#define BL_STORE_LOCAL_DATA_IN_EEPROM  0x92
#define BL_UPLOAD_AUTO_SCSI_CODE  0x94
#define BL_MODIFY_IO_ADDRESS  0x95
#define BL_SET_CCB_FORMAT  0x96
#define BL_WRITE_INQUIRY_BUFFER  0x9a
#define BL_READ_INQUIRY_BUFFER  0x9b
#define BL_FLASH_ROM_UPLOAD_DOWNLOAD  0xa7
#define BL_READ_SCAM_DATA  0xa8
#define BL_WRITE_SCAM_DATA  0xa9



#define BL_COMMAND_SIZE_MAX   53
#define BL_REPLY_SIZE_MAX     64

#define BL_NOOP 0xff


 /**
 * Action codes for outgoing mailboxes.
 */
#define BUSLOGIC_MAILBOX_OUTGOING_ACTION_FREE 0x00
#define BUSLOGIC_MAILBOX_OUTGOING_ACTION_START_COMMAND 0x01
#define BUSLOGIC_MAILBOX_OUTGOING_ACTION_ABORT_COMMAND 0x02


/**
 * Completion codes for incoming mailboxes.
 */

#define BUSLOGIC_MAILBOX_INCOMING_COMPLETION_FREE 0x00
#define BUSLOGIC_MAILBOX_INCOMING_COMPLETION_WITHOUT_ERROR 0x01
#define BUSLOGIC_MAILBOX_INCOMING_COMPLETION_ABORTED 0x02
#define BUSLOGIC_MAILBOX_INCOMING_COMPLETION_ABORTED_NOT_FOUND 0x03
#define BUSLOGIC_MAILBOX_INCOMING_COMPLETION_WITH_ERROR 0x04
#define BUSLOGIC_MAILBOX_INCOMING_COMPLETION_INVALID_CCB 0x05

 /**
 * Host adapter status for incoming mailboxes.
 */

#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_CMD_COMPLETED 0x00
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_LINKED_CMD_COMPLETED 0x0a
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_LINKED_CMD_COMPLETED_WITH_FLAG 0x0b
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_DATA_UNDERUN 0x0
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_SCSI_SELECTION_TIMEOUT 0x11
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_DATA_OVERRUN 0x12
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_UNEXPECTED_BUS_FREE 0x13
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_INVALID_BUS_PHASE_REQUESTED 0x14
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_INVALID_OUTGOING_MAILBOX_ACTION_CODE 0x15
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_INVALID_COMMAND_OPERATION_CODE 0x16
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_LINKED_CCB_HAS_INVALID_LUN 0x17
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_INVALID_COMMAND_PARAMETER 0x1a
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_AUTO_REQUEST_SENSE_FAILED 0x1b
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_TAGGED_QUEUING_MESSAGE_REJECTED 0x1c
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_UNSUPPORTED_MESSAGE_RECEIVED 0x1d
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_HOST_ADAPTER_HARDWARE_FAILED 0x20
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_TARGET_FAILED_RESPONSE_TO_ATN 0x21
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_HOST_ADAPTER_ASSERTED_RST 0x22
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_OTHER_DEVICE_ASSERTED_RST 0x23
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_TARGET_DEVICE_RECONNECTED_IMPROPERLY 0x24
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_HOST_ADAPTER_ASSERTED_BUS_DEVICE_RESET 0x25
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_ABORT_QUEUE_GENERATED 0x26
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_HOST_ADAPTER_SOFTWARE_ERROR 0x27
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_HOST_ADAPTER_HARDWARE_TIMEOUT_ERROR 0x30
#define BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_SCSI_PARITY_ERROR_DETECTED 0x34

/**
 * Device status codes for incoming mailboxes.
 */

#define BUSLOGIC_MAILBOX_INCOMING_DEVICE_STATUS_OPERATION_GOOD 0x00
#define BUSLOGIC_MAILBOX_INCOMING_DEVICE_STATUS_CHECK_CONDITION 0x02
#define BUSLOGIC_MAILBOX_INCOMING_DEVICE_STATUS_DEVICE_BUSY 0x08


# define RT_OFFSETOF(type, member)              ( (int)(uintptr_t)&( ((type *)(void *)0)->member) )

 /**
 * Data transfer direction.
 */

#define BUSLOGIC_CCB_DIRECTION_UNKNOWN 0x00
#define BUSLOGIC_CCB_DIRECTION_IN 0x01
#define BUSLOGIC_CCB_DIRECTION_OUT 0x02
#define BUSLOGIC_CCB_DIRECTION_NO_DATA 0x03


 /**
 * Opcode types for CCB.
 */

#define BUSLOGIC_CCB_OPCODE_INITIATOR_CCB 0x00
#define BUSLOGIC_CCB_OPCODE_TARGET_CCB 0x01
#define BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_SCATTER_GATHER 0x02
#define BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_RESIDUAL_DATA_LENGTH 0x03
#define BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_RESIDUAL_SCATTER_GATHER 0x04
#define BUSLOGIC_CCB_OPCODE_BUS_DEVICE_RESET 0x81



 /**
 * Data direction.
 */
#define PDMSCSIREQUESTTXDIR_UNKNOWN      0x00
#define PDMSCSIREQUESTTXDIR_FROM_DEVICE  0x01
#define PDMSCSIREQUESTTXDIR_TO_DEVICE    0x02
#define PDMSCSIREQUESTTXDIR_NONE         0x03
#define PDMSCSIREQUESTTXDIR_32BIT_HACK   0x7fffffff


typedef struct SCSIDiskReq {
    SCSIRequest req;
    /* Both sector and sector_count are in terms of qemu 512 byte blocks.  */
    uint64_t sector;
    uint32_t sector_count;
    uint32_t buflen;
    bool started;
    struct iovec iov;
    QEMUIOVector qiov;
    BlockAcctCookie acct;
} SCSIDiskReq;

// struct
// {
//     QTAILQ_HEAD(,);
// } queue;

 /**
 * State of a device attached to the buslogic host adapter.
 *
 * @implements  PDMIBASE
 * @implements  PDMISCSIPORT
 * @implements  PDMILEDPORTS
 */
typedef struct BUSLOGICDEVICE
{
    struct BuslogicState *pBusLogic;
    /** Flag whether device is present. */
    bool                          fPresent;
    /** LUN of the device. */
    uint32_t                        iLUN;
    uint32_t                    id;
    uint32_t                    channel;

// #if HC_ARCH_BITS == 64
//     uint32_t                      Alignment0;
// #endif

    /** Our base interface. */
    // PDMIBASE                      IBase;
    /** SCSI port interface. */
    // PDMISCSIPORT                  ISCSIPort;
    /** Led interface. */
    // PDMILEDPORTS                  ILed;
    /** Pointer to the attached driver's base interface. */
    // R3PTRTYPE(PPDMIBASE)          pDrvBase;
    /** Pointer to the underlying SCSI connector interface. */
    // R3PTRTYPE(PPDMISCSICONNECTOR) pDrvSCSIConnector;
    /** The status LED state for this device. */
    // PDMLED                        Led;

// #if HC_ARCH_BITS == 64
//     uint32_t                      Alignment1;
// #endif

    /** Number of outstanding tasks on the port. */
    volatile uint32_t             cOutstandingRequests;

} BUSLOGICDEVICE;


/** Structure for the INQUIRE_PCI_HOST_ADAPTER_INFORMATION reply. */
typedef struct ReplyInquirePCIHostAdapterInformation
{
    uint8_t       IsaIOPort;
    uint8_t       IRQ;
    unsigned char LowByteTerminated : 1;
    unsigned char HighByteTerminated : 1;
    unsigned char uReserved : 2; /* Reserved. */
    unsigned char JP1 : 1; /* Whatever that means. */
    unsigned char JP2 : 1; /* Whatever that means. */
    unsigned char JP3 : 1; /* Whatever that means. */
    /** Whether the provided info is valid. */
    unsigned char InformationIsValid: 1;
    uint8_t       uReserved2; /* Reserved. */
}QEMU_PACKED ReplyInquirePCIHostAdapterInformation;

 /** Ugly 24-bit big-endian addressing. */
typedef struct
{
    uint8_t hi;
    uint8_t mid;
    uint8_t lo;
} Addr24, Len24;

#define ADDR_TO_U32(x)      (((x).hi << 16) | ((x).mid << 8) | (x).lo)
#define LEN_TO_U32          ADDR_TO_U32
#define U32_TO_ADDR(a, x)   do {(a).hi = (x) >> 16; (a).mid = (x) >> 8; (a).lo = (x);} while(0)
#define U32_TO_LEN          U32_TO_ADDR

/**
 * Auto SCSI structure which is located
 * in host adapter RAM and contains several
 * configuration parameters.
 */
typedef struct AutoSCSIRam
{
    uint8_t       aInternalSignature[2];
    uint8_t       cbInformation;
    uint8_t       aHostAdaptertype[6];
    uint8_t       uReserved1;
    bool          fFloppyEnabled :                  1;
    bool          fFloppySecondary :                1;
    bool          fLevelSensitiveInterrupt :        1;
    unsigned char uReserved2 :                      2;
    unsigned char uSystemRAMAreForBIOS :            3;
    unsigned char uDMAChannel :                     7;
    bool          fDMAAutoConfiguration :           1;
    unsigned char uIrqChannel :                     7;
    bool          fIrqAutoConfiguration :           1;
    uint8_t       uDMATransferRate;
    uint8_t       uSCSIId;
    bool          fLowByteTerminated :              1;
    bool          fParityCheckingEnabled :          1;
    bool          fHighByteTerminated :             1;
    bool          fNoisyCablingEnvironment :        1;
    bool          fFastSynchronousNeogtiation :     1;
    bool          fBusResetEnabled :                1;
    bool          fReserved3 :                      1;
    bool          fActiveNegotiationEnabled :       1;
    uint8_t       uBusOnDelay;
    uint8_t       uBusOffDelay;
    bool          fHostAdapterBIOSEnabled :         1;
    bool          fBIOSRedirectionOfInt19 :         1;
    bool          fExtendedTranslation :            1;
    bool          fMapRemovableAsFixed :            1;
    bool          fReserved4 :                      1;
    bool          fBIOSSupportsMoreThan2Drives :    1;
    bool          fBIOSInterruptMode :              1;
    bool          fFlopticalSupport :               1;
    uint16_t      u16DeviceEnabledMask;
    uint16_t      u16WidePermittedMask;
    uint16_t      u16FastPermittedMask;
    uint16_t      u16SynchronousPermittedMask;
    uint16_t      u16DisconnectPermittedMask;
    uint16_t      u16SendStartUnitCommandMask;
    uint16_t      u16IgnoreInBIOSScanMask;
    unsigned char uPCIInterruptPin :                2;
    unsigned char uHostAdapterIoPortAddress :       2;
    bool          fStrictRoundRobinMode :           1;
    bool          fVesaBusSpeedGreaterThan33MHz :   1;
    bool          fVesaBurstWrite :                 1;
    bool          fVesaBurstRead :                  1;
    uint16_t      u16UltraPermittedMask;
    uint32_t      uReserved5;
    uint8_t       uReserved6;
    uint8_t       uAutoSCSIMaximumLUN;
    bool          fReserved7 :                      1;
    bool          fSCAMDominant :                   1;
    bool          fSCAMenabled :                    1;
    bool          fSCAMLevel2 :                     1;
    unsigned char uReserved8 :                      4;
    bool          fInt13Extension :                 1;
    bool          fReserved9 :                      1;
    bool          fCDROMBoot :                      1;
    unsigned char uReserved10 :                     5;
    unsigned char uBootTargetId :                   4;
    unsigned char uBootChannel :                    4;
    bool          fForceBusDeviceScanningOrder :    1;
    unsigned char uReserved11 :                     7;
    uint16_t      u16NonTaggedToAlternateLunPermittedMask;
    uint16_t      u16RenegotiateSyncAfterCheckConditionMask;
    uint8_t       aReserved12[10];
    uint8_t       aManufacturingDiagnostic[2];
    uint16_t      u16Checksum;
}QEMU_PACKED AutoSCSIRam;

/**
 * The local Ram.
 */
typedef union HostAdapterLocalRam
{
    /** Byte view. */
    uint8_t u8View[256];
    /** Structured view. */
    struct
    {
        /** Offset 0 - 63 is for BIOS. */
        uint8_t     u8Bios[64];
        /** Auto SCSI structure. */
        AutoSCSIRam autoSCSIData;
    } structured;
}QEMU_PACKED HostAdapterLocalRam;






 /** Structure for the INITIALIZE MAILBOX request. */
typedef struct
{
    /** Number of mailboxes to set up. */
    uint8_t     cMailbox;
    /* Physical address of the first mailbox.*/
    Addr24      aMailboxBaseAddr;
}QEMU_PACKED RequestInitMbx;


/** Old style 24-bit mailbox entry. */
typedef struct Mailbox24
{
    /** Mailbox command (incoming) or state (outgoing). */
    uint8_t     uCmdState;
    /** Physical address of the CCB structure in the guest memory. */
    Addr24      aPhysAddrCCB;
} Mailbox24;

/**
 * Structure of a mailbox in guest memory.
 * The incoming and outgoing mailbox have the same size
 * but the incoming one has some more fields defined which
 * are marked as reserved in the outgoing one.
 * The last field is also different from the type.
 * For outgoing mailboxes it is the action and
 * for incoming ones the completion status code for the task.
 * We use one structure for both types.
 */
typedef struct Mailbox32
{
    /** Physical address of the CCB structure in the guest memory. */
    uint32_t u32PhysAddrCCB;
    /** Type specific data. */
    union
    {
        /** For outgoing mailboxes. */
        struct
        {
            /** Reserved */
            uint8_t uReserved[3];
            /** Action code. */
            uint8_t uActionCode;
        } out;
        /** For incoming mailboxes. */
        struct
        {
            /** The host adapter status after finishing the request. */
            uint8_t  uHostAdapterStatus;
            /** The status of the device which executed the request after executing it. */
            uint8_t  uTargetDeviceStatus;
            /** Reserved. */
            uint8_t  uReserved;
            /** The completion status code of the request. */
            uint8_t uCompletionCode;
        } in;
    } u;
} Mailbox32;




typedef struct RequestInitializeExtendedMailbox
{
    /** Number of mailboxes in guest memory. */
    uint8_t  cMailbox;
    /** Physical address of the first mailbox. */
    uint32_t uMailboxBaseAddress;
}QEMU_PACKED RequestInitializeExtendedMailbox;


typedef struct ReplyInquireExtendedSetupInformation
{
    uint8_t       uBusType;
    uint8_t       uBiosAddress;
    uint16_t      u16ScatterGatherLimit;
    uint8_t       cMailbox;
    uint32_t      uMailboxAddressBase;
    unsigned char uReserved1 : 2;
    bool          fFastEISA : 1;
    unsigned char uReserved2 : 3;
    bool          fLevelSensitiveInterrupt : 1;
    unsigned char uReserved3 : 1;
    unsigned char aFirmwareRevision[3];
    bool          fHostWideSCSI : 1;
    bool          fHostDifferentialSCSI : 1;
    bool          fHostSupportsSCAM : 1;
    bool          fHostUltraSCSI : 1;
    bool          fHostSmartTermination : 1;
    unsigned char uReserved4 : 3;
}QEMU_PACKED ReplyInquireExtendedSetupInformation;

/** Structure for the INQUIRE_SETUP_INFORMATION reply. */
typedef struct ReplyInquireSetupInformationSynchronousValue
{
    unsigned char uOffset :         4;
    unsigned char uTransferPeriod : 3;
    bool fSynchronous :             1;
}QEMU_PACKED ReplyInquireSetupInformationSynchronousValue;



typedef struct ReplyInquireSetupInformation
{
    bool fSynchronousInitiationEnabled : 1;
    bool fParityCheckingEnabled :        1;
    unsigned char uReserved1 :           6;
    uint8_t uBusTransferRate;
    uint8_t uPreemptTimeOnBus;
    uint8_t uTimeOffBus;
    uint8_t cMailbox;
    Addr24  MailboxAddress;
    ReplyInquireSetupInformationSynchronousValue SynchronousValuesId0To7[8];
    uint8_t uDisconnectPermittedId0To7;
    uint8_t uSignature;
    uint8_t uCharacterD;
    uint8_t uHostBusType;
    uint8_t uWideTransferPermittedId0To7;
    uint8_t uWideTransfersActiveId0To7;
    ReplyInquireSetupInformationSynchronousValue SynchronousValuesId8To15[8];
    uint8_t uDisconnectPermittedId8To15;
    uint8_t uReserved2;
    uint8_t uWideTransferPermittedId8To15;
    uint8_t uWideTransfersActiveId8To15;
}QEMU_PACKED ReplyInquireSetupInformation;



/** Structure for the INQUIRE_CONFIGURATION reply. */
typedef struct ReplyInquireConfiguration
{
    unsigned char uReserved1 :     5;
    bool          fDmaChannel5 :   1;
    bool          fDmaChannel6 :   1;
    bool          fDmaChannel7 :   1;
    bool          fIrqChannel9 :   1;
    bool          fIrqChannel10 :  1;
    bool          fIrqChannel11 :  1;
    bool          fIrqChannel12 :  1;
    unsigned char uReserved2 :     1;
    bool          fIrqChannel14 :  1;
    bool          fIrqChannel15 :  1;
    unsigned char uReserved3 :     1;
    unsigned char uHostAdapterId : 4;
    unsigned char uReserved4 :     4;
}QEMU_PACKED ReplyInquireConfiguration;


typedef struct bl_request {
    SCSIRequest *req;
    uint32_t tag;
    uint32_t dma_len;
    uint8_t *dma_buf;
    uint32_t pending;
    int out;
    QTAILQ_ENTRY(bl_request) next;
} bl_request;


typedef struct PCIHostAdapterInformation
{
    uint8_t       IsaIOPort;
    uint8_t       IRQ;
    //byte field [0] = LowByteTerminated
    // [1] = HighByteTerminated
    // [2-3] = uReserved
    // [4] = JP1
    // [5] = JP2
    // [6] = JP3
    // [7] = InformationIsValid
    // unsigned char LowByteTerminated : 1;
    // unsigned char HighByteTerminated : 1;
    // unsigned char uReserved : 2; /* Reserved. */
    // unsigned char JP1 : 1; /* Whatever that means. */
    // unsigned char JP2 : 1; /* Whatever that means. */
    // unsigned char JP3 : 1; /* Whatever that means. */
    // /** Whether the provided info is valid. */
    // unsigned char InformationIsValid: 1;
    uint8_t         byteField;
    uint8_t       uReserved2; /* Reserved. */
} PCIHostAdapterInformation;

typedef struct BUSLOGICTASKSTATE BUSLOGICTASKSTATE;


typedef struct {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    MemoryRegion port_io;
    MemoryRegion ram_io;
    MemoryRegion mmio_io;

    uint8_t status_register;
    uint8_t data_in_register;
    uint8_t interrupt_register;
    uint8_t geo_register;

    bool is_status_read;

    uint8_t current_command;

    //command from the host
    uint8_t command_bytes_left;
    uint8_t current_command_byte;
    uint8_t command_buffer[BL_COMMAND_SIZE_MAX];

    //reply to the host
    uint8_t reply_bytes_left;
    uint8_t current_reply_byte;
    uint8_t reply_buffer[BL_REPLY_SIZE_MAX];

    HostAdapterLocalRam localRam;
    uint8_t uPendingIntr;

    /** Number of mailboxes the guest set up. */
    uint32_t                        mailbox;

    /** Flag whether 24-bit mailboxes are in use (default is 32-bit). */
    bool                            fMbxIs24Bit;

    /** Physical base address of the outgoing mailboxes. */
    uint64_t                        GCPhysAddrMailboxOutgoingBase;
    /** Current outgoing mailbox position. */
    uint32_t                        uMailboxOutgoingPositionCurrent;
    /** Number of mailboxes ready. */
    volatile uint32_t               cMailboxesReady;
    /** Whether a notification to R3 was send. */
    volatile bool                   fNotificationSend;

// #if HC_ARCH_BITS == 64
//     uint32_t                        Alignment1;
// #endif

    /** Physical base address of the incoming mailboxes. */
    uint64_t                        GCPhysAddrMailboxIncomingBase;
    /** Current incoming mailbox position. */
    uint32_t                        uMailboxIncomingPositionCurrent;
    /** ISA I/O port base (encoded in FW-compatible format). */
    uint8_t                         uISABaseCode;
    /** Whether we are using the RAM or reply buffer. */
    bool                            fUseLocalRam;
    /** Whether strict round robin is enabled. */
    bool                            fStrictRoundRobinMode;
    /** Whether the extended LUN CCB format is enabled for 32 possible logical units. */
    bool                            fExtendedLunCCBFormat;
    /** BusLogic device states. */
    BUSLOGICDEVICE                 deviceStates[BUSLOGIC_MAX_DEVICES];
    /** Flag whether IRQs are enabled. */
    bool                            fIRQEnabled;



    // PCIHostAdapterInformation PHAD_INFO;
    BUSLOGICTASKSTATE           *current_task;


    // int carry; /* ??? Should this be an a visible register somewhere?  */
    // int status;
    // /* Action to take at the end of a MSG IN phase.
    //    0 = COMMAND, 1 = disconnect, 2 = DATA OUT, 3 = DATA IN.  */
    // int msg_action;
    // int msg_len;
    // uint8_t msg[LSI_MAX_MSGIN_LEN];
    //  0 if SCRIPTS are running or stopped.
    //  * 1 if a Wait Reselect instruction has been issued.
    //  * 2 if processing DMA from lsi_execute_script.
    //  * 3 if a DMA operation is in progress.
    // int waiting;
    SCSIBus bus;
    // int current_lun;
    // /* The tag is a combination of the device ID and the SCSI tag.  */
    // uint32_t select_tag;
    // int command_complete;
    QTAILQ_HEAD(, bl_request) queue;
    // lsi_request *current;
    QEMUTimer *mailboxCheckTimer;

} BuslogicState;


/**
 * A S/G entry.
 */
typedef struct RTSGSEG
{
    /** Pointer to the segment buffer. */
    void   *pvSeg;
    /** Size of the segment buffer. */
    size_t  cbSeg;
} RTSGSEG;



/**
 * The command control block for a SCSI request.
 */
typedef struct CCB32
{
    /** Opcode. */
    uint8_t       uOpcode;
    /** Reserved */
    unsigned char uReserved1 :      3;
    /** Data direction for the request. */
    unsigned char uDataDirection :  2;
    /** Whether the request is tag queued. */
    bool          fTagQueued :      1;
    /** Queue tag mode. */
    unsigned char uQueueTag :       2;
    /** Length of the SCSI CDB. */
    uint8_t       cbCDB;
    /** Sense data length. */
    uint8_t       cbSenseData;
    /** Data length. */
    uint32_t      cbData;
    /** Data pointer.
     *  This points to the data region or a scatter gather list based on the opcode.
     */
    uint32_t      u32PhysAddrData;
    /** Reserved. */
    uint8_t       uReserved2[2];
    /** Host adapter status. */
    uint8_t       uHostAdapterStatus;
    /** Device adapter status. */
    uint8_t       uDeviceStatus;
    /** The device the request is sent to. */
    uint8_t       uTargetId;
    /**The LUN in the device. */
    unsigned char uLogicalUnit : 5;
    /** Legacy tag. */
    bool          fLegacyTagEnable : 1;
    /** Legacy queue tag. */
    unsigned char uLegacyQueueTag : 2;
    /** The SCSI CDB.  (A CDB can be 12 bytes long.) */
    uint8_t       abCDB[12];
    /** Reserved. */
    uint8_t       uReserved3[6];
    /** Sense data pointer. */
    uint32_t      u32PhysAddrSenseData;
}QEMU_PACKED CCB32;


/**
 * The 24-bit command control block.
 */
typedef struct CCB24
{
    /** Opcode. */
    uint8_t         uOpcode;
    /** The LUN in the device. */
    unsigned char   uLogicalUnit : 3;
    /** Data direction for the request. */
    unsigned char   uDataDirection : 2;
    /** The target device ID. */
    unsigned char   uTargetId : 3;
    /** Length of the SCSI CDB. */
    uint8_t         cbCDB;
    /** Sense data length. */
    uint8_t         cbSenseData;
    /** Data length. */
    Len24           acbData;
    /** Data pointer.
     *  This points to the data region or a scatter gather list based on the opc
     */
    Addr24          aPhysAddrData;
    /** Pointer to next CCB for linked commands. */
    Addr24          aPhysAddrLink;
    /** Command linking identifier. */
    uint8_t         uLinkId;
    /** Host adapter status. */
    uint8_t         uHostAdapterStatus;
    /** Device adapter status. */
    uint8_t         uDeviceStatus;
    /** Two unused bytes. */
    uint8_t         aReserved[2];
    /** The SCSI CDB.  (A CDB can be 12 bytes long.)   */
    uint8_t         abCDB[12];
}QEMU_PACKED CCB24;


/**
 * The common 24-bit/32-bit command control block. The 32-bit CCB is laid out
 * such that many fields are in the same location as in the older 24-bit CCB.
 */
typedef struct CCBC
{
    /** Opcode. */
    uint8_t         uOpcode;
    /** The LUN in the device. */
    unsigned char   uPad1 : 3;
    /** Data direction for the request. */
    unsigned char   uDataDirection : 2;
    /** The target device ID. */
    unsigned char   uPad2 : 3;
    /** Length of the SCSI CDB. */
    uint8_t         cbCDB;
    /** Sense data length. */
    uint8_t         cbSenseData;
    uint8_t         aPad1[10];
    /** Host adapter status. */
    uint8_t         uHostAdapterStatus;
    /** Device adapter status. */
    uint8_t         uDeviceStatus;
    uint8_t         aPad2[2];
    /** The SCSI CDB (up to 12 bytes). */
    uint8_t         abCDB[12];
}QEMU_PACKED CCBC;


/** A union of all CCB types (24-bit/32-bit/common). */
typedef union CCBU
{
    CCB32    n;     /**< New 32-bit CCB. */
    CCB24    o;     /**< Old 24-bit CCB. */
    CCBC     c;     /**< Common CCB subset. */
} CCBU;


/**
 * Task state for a CCB request.
 */
typedef struct BUSLOGICTASKSTATE
{
    /** Next in the redo list. */
    BUSLOGICTASKSTATE             *RedoNext;
    /** Device this task is assigned to. */
    BUSLOGICDEVICE     *TargetDevice;
    /** The command control block from the guest. */
    CCBU                CommandControlBlockGuest;
    /** Mailbox read from guest memory. */
    Mailbox32           MailboxGuest;
    /** The SCSI request we pass to the underlying SCSI engine. */
    SCSIRequest         *pScsiRequest;
    /** Data buffer segment */
    // RTSGSEG             DataSeg;
    ScatterGatherEntry         sglEntry;
    QEMUSGList          *sglist;

    uint8_t             *dataBuffer;
    /** Pointer to the R3 sense buffer. */
    uint8_t            *pbSenseBuffer;
    /** Flag whether this is a request from the BIOS. */
    bool                fBIOS;
    /** 24-bit request flag (default is 32-bit). */
    bool                fIs24Bit;
    /** S/G entry size (depends on the above flag). */
    uint8_t             cbSGEntry;
} BUSLOGICTASKSTATE;


/** 32-bit scatter-gather list entry. */
typedef struct SGE32
{
    uint32_t   cbSegment;
    uint32_t   u32PhysAddrSegmentBase;
}QEMU_PACKED SGE32;


/** 24-bit scatter-gather list entry. */
typedef struct SGE24
{
    Len24       acbSegment;
    Addr24      aPhysAddrSegmentBase;
}QEMU_PACKED SGE24;




static const VMStateDescription vmstate_buslogic_devices = {
    .name = "BUSLOGICDEVICE",
    .version_id = 0,
    .minimum_version_id = 0,
    // .pre_save = lsi_pre_save,
    .fields = (VMStateField[]) {
        // VMSTATE_UINT32(pBusLogic, BUSLOGICDEVICE),
        VMSTATE_BOOL(fPresent, BUSLOGICDEVICE),
        VMSTATE_UINT32(iLUN, BUSLOGICDEVICE),
        VMSTATE_UINT32(id, BUSLOGICDEVICE),
        VMSTATE_UINT32(channel, BUSLOGICDEVICE),
        VMSTATE_UINT32(cOutstandingRequests, BUSLOGICDEVICE),

        VMSTATE_END_OF_LIST()
    }
};




//static const VMStateDescription vmstate_buslogic_scsi = {
//    .name = "buslogicscsi",
//    .version_id = 0,
//    .minimum_version_id = 0,
//    // .pre_save = lsi_pre_save,
//    .fields = (VMStateField[]) {
//        VMSTATE_PCI_DEVICE(parent_obj, BuslogicState),
//
//        VMSTATE_UINT8(status_register, BuslogicState),
//        VMSTATE_UINT8(data_in_register, BuslogicState),
//        VMSTATE_UINT8(interrupt_register, BuslogicState),
//        VMSTATE_UINT8(geo_register, BuslogicState),
//        VMSTATE_BOOL(is_status_read, BuslogicState),
//        VMSTATE_UINT8(current_command, BuslogicState),
//        VMSTATE_UINT8(command_bytes_left, BuslogicState),
//        VMSTATE_UINT8(current_command_byte, BuslogicState),
//        VMSTATE_BUFFER(command_buffer, BuslogicState),
//        VMSTATE_UINT8(reply_bytes_left, BuslogicState),
//        VMSTATE_UINT8(current_reply_byte, BuslogicState),
//        VMSTATE_BUFFER(reply_buffer, BuslogicState),
//        VMSTATE_UINT32(mailbox, BuslogicState),
//        VMSTATE_BOOL(fMbxIs24Bit, BuslogicState),
//        VMSTATE_UINT64(GCPhysAddrMailboxOutgoingBase, BuslogicState),
//        VMSTATE_UINT32(uMailboxOutgoingPositionCurrent, BuslogicState),
//        VMSTATE_UINT32(cMailboxesReady, BuslogicState),
//        VMSTATE_BOOL(fNotificationSend, BuslogicState),
//        VMSTATE_UINT64(GCPhysAddrMailboxIncomingBase, BuslogicState),
//        VMSTATE_UINT32(uMailboxIncomingPositionCurrent, BuslogicState),
//        VMSTATE_UINT8(uISABaseCode, BuslogicState),
//        VMSTATE_BOOL(fUseLocalRam, BuslogicState),
//        VMSTATE_BOOL(fStrictRoundRobinMode, BuslogicState),
//        VMSTATE_BOOL(fExtendedLunCCBFormat, BuslogicState),
//        VMSTATE_STRUCT_ARRAY(deviceStates, BuslogicState, BUSLOGIC_MAX_DEVICES, 0, vmstate_buslogic_devices, BUSLOGICDEVICE),
//        VMSTATE_BOOL(fIRQEnabled, BuslogicState),
//	    VMSTATE_TIMER(*mailboxCheckTimer, BuslogicState),
//        VMSTATE_END_OF_LIST()
//    }
//};
//




void buslogic_hard_reset(BuslogicState *s);
void buslogic_soft_reset(BuslogicState *s);
void buslogic_int_reset(BuslogicState *s);
void buslogic_clear_interrupt(BuslogicState *s);

uint8_t buslogic_process_command(BuslogicState *s);
void buslogic_command_complete(BuslogicState *s,  bool supressIrq);
void buslogic_set_interrupt(BuslogicState *s, bool supressIrq, uint8_t irqType);
void buslogic_check_mailboxes(void *opaque);
void buslogicSendIncomingMailbox(BuslogicState *pBusLogic, BUSLOGICTASKSTATE *pTaskState,uint8_t uHostAdapterStatus, uint8_t uDeviceStatus,uint8_t uMailboxCompletionCode);
void buslogicDataBufferFree(BUSLOGICTASKSTATE *pTaskState);









void buslogic_set_interrupt(BuslogicState *s, bool supressIrq, uint8_t irqType)
{
    // LogFlowFunc(("pBusLogic=%#p\n", pBusLogic));
    PCIDevice *d = PCI_DEVICE(s);

    /* The CMDC interrupt has priority over IMBL and OMBR. */
    if (irqType & (BL_INTR_IMBL | BL_INTR_OMBR))
    {
        if (!(s->interrupt_register & BL_INTR_CMDC))
            s->interrupt_register |= irqType;    /* Report now. */
        else
        {
            qemu_log_mask(LOG_GUEST_ERROR, "pending interrupt\n");
            s->uPendingIntr |= irqType;    /* Report later. */
        }
    }
    else if (irqType & BL_INTR_CMDC)
    {
        s->interrupt_register |= irqType;
    }
    else
        qemu_log_mask(LOG_GUEST_ERROR, "invalid interrupt\n");

    s->interrupt_register |= BL_INTR_INTV;
    if (s->fIRQEnabled && !supressIrq)
    {
        qemu_log_mask(LOG_GUEST_ERROR, "IRQ!!!!!!!!\n");
        pci_set_irq(d, 1);
    }
}


void buslogic_command_complete(BuslogicState *s, bool supressIrq)
{

    qemu_log_mask(LOG_GUEST_ERROR, "command complete %02x\n",s->current_command);

    s->fUseLocalRam = false;
    s->status_register |= BL_STAT_HARDY;
    s->reply_bytes_left = 0;
    s->current_reply_byte = 0;

    s->command_bytes_left = 0;
    s->current_command_byte = 0;

    /* Modify I/O address does not generate an interrupt. */
    if (s->current_command != BL_EXECUTE_MAILBOX_COMMAND)
    {
        /* Notify that the command is complete. */
        s->status_register &= ~BL_STAT_DIRRDY;
        buslogic_set_interrupt(s, supressIrq, BL_INTR_CMDC);
    }

    s->current_command = BL_NOOP;
    s->current_command_byte = 0;
}


static void buslogic_port_write(void *opaque, hwaddr addr,
                           uint64_t val, unsigned size)
{
    BuslogicState *s = opaque;
    switch (addr)
    {
        case BL_CTRL_WREG:
        {
            if (val & BL_CTRL_RHARD)
            {
                qemu_log_mask(LOG_GUEST_ERROR, "%s opa=%p addr=%li val=0x%"PRIx64" size=%i BL_CTRL_RHARD\n", __FUNCTION__, opaque, addr, val, size);
                // s->status_register &= ~BL_STAT_HARDY;
                buslogic_hard_reset(s);
            }
            else if (val & BL_CTRL_RSOFT)
            {
                qemu_log_mask(LOG_GUEST_ERROR, "%s opa=%p addr=%li val=0x%"PRIx64" size=%i BL_CTRL_RSOFT\n", __FUNCTION__, opaque, addr, val, size);
                buslogic_soft_reset(s);
            }
            else if (val & BL_CTRL_RINT)
            {
                qemu_log_mask(LOG_GUEST_ERROR, "%s opa=%p addr=%li val=0x%"PRIx64" size=%i BL_CTRL_RINT\n", __FUNCTION__, opaque, addr, val, size);
                buslogic_int_reset(s);
            }
            break;
        }

        case BL_CMD_WREG:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "%s opa=%p addr=%li val=0x%02x size=%i CMD_REG\n", __FUNCTION__, opaque, addr, (char)val, size);

            /* Fast path for mailbox execution command. */
            if ((val == BL_EXECUTE_MAILBOX_COMMAND) && (s->current_command == BL_NOOP))
            {
                /* If there are no mailboxes configured, don't even try to do anything. */
                if (s->mailbox)
                {
                    s->cMailboxesReady++;

                    s->fNotificationSend = true;
                }

                // return rc;
                return;
            }


            if (s->current_command == BL_NOOP)
            {
                //save code of current command
                s->current_command = val;
                //clear count of input parameters
                s->command_bytes_left = 0;
                s->current_command_byte = 0;

                s->reply_bytes_left = 0;
                s->current_reply_byte = 0;
                //mark adapter as busy
                s->status_register &= ~BL_STAT_HARDY;

                switch (s->current_command)
                {
                    case BL_TEST_CMDC_INTERRUPT:
                    case BL_INQUIRE_FIRMWARE_VERSION_LETTER:
                    case BL_INQUIRE_BOARD_ID:
                    case BL_INQUIRE_FIRMWARE_VERSION_3RD_LETTER:
                    case BL_INQUIRE_PCI_HOST_ADAPTER_INFORMATION:
                    case BL_INQUIRE_CONFIGURATION:
                    case BL_INQUIRE_INSTALLED_DEVICES_ID_0_TO_7:
                    case BL_INQUIRE_INSTALLED_DEVICES_ID_8_TO_15:
                    case BL_INQUIRE_TARGET_DEVICES:
                        s->command_bytes_left = 0;
                        break;
                    case BL_MODIFY_IO_ADDRESS:
                    case BL_INQUIRE_EXTENDED_SETUP_INFORMATION:
                    case BL_INQUIRE_SETUP_INFORMATION:
                    case BL_INQUIRE_HOST_ADAPTER_MODEL_NUMBER:
                    case BL_ENABLE_STRICT_ROUND_ROBIN_MODE:
                    case BL_SET_CCB_FORMAT:
                    case BL_INQUIRE_SYNCHRONOUS_PERIOD:
                    case BL_DISABLE_HOST_ADAPTER_INTERRUPT:
                    case BL_ECHO_COMMAND_DATA:
                    case BL_SET_PREEMPT_TIME_ON_BUS:
                    case BL_SET_TIME_OFF_BUS:
                    case BL_SET_BUS_TRANSFER_RATE:
                        s->command_bytes_left = 1;
                        break;
                    case BL_FETCH_HOST_ADAPTER_LOCAL_RAM:
                        s->command_bytes_left = 2;
                        break;
                    case BL_READ_BUSMASTER_CHIP_FIFO:
                    case BL_WRITE_BUSMASTER_CHIP_FIFO:
                        s->command_bytes_left = 3;
                        break;
                    case BL_INITIALIZE_MAILBOX:
                        s->command_bytes_left = sizeof(RequestInitMbx);
                        break;
                    case BL_INITIALIZE_EXTENDED_MAILBOX:
                        s->command_bytes_left = sizeof(RequestInitializeExtendedMailbox);
                        break;
                    case BL_SET_ADAPTER_OPTIONS:
                        /* There must be at least one byte following this command. */
                        s->command_bytes_left = 1;
                        break;
                    case BL_EXECUTE_SCSI_COMMAND:
                        /* 12 bytes + variable-length CDB. */
                        s->command_bytes_left = 12;
                        break;
                    case BL_EXT_BIOS_INFO:
                    case BL_UNLOCK_MAILBOX:
                        /* Invalid commands. */
                        s->command_bytes_left = 0;
                        break;
                    case BL_EXECUTE_MAILBOX_COMMAND: /* Should not come here anymore. */
                    default:
                        // AssertMsgFailed(("Invalid operation code %#x\n", uVal));
                        qemu_log_mask(LOG_GUEST_ERROR, "invalid operation code \n");
                }
            }
            else
            {
                s->command_buffer[s->current_command_byte] = val;
                s->current_command_byte++;
                s->command_bytes_left--;
            }
            if (!s->command_bytes_left)
            {
                buslogic_process_command(s);
            }

            break;
        }
        case BL_GEO_WREG:
        {
            break;
        }


        default:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "ATTENTION NEW TYPE OF WRITE REGISTER = %li\n",addr);
            break;
        }
    }
}

static uint64_t buslogic_port_read(void *opaque, hwaddr addr,
                              unsigned size)
{
    uint64_t result;
    // qemu_log_mask(LOG_GUEST_ERROR, "READ\n");
    BuslogicState *s = opaque;
    switch (addr)
    {
        case BL_STAT_RREG:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "%s BL_STAT_RREG size= %i s->status_register = 0x%02x\n",__FUNCTION__,size, s->status_register);

            if (s->status_register & BL_STAT_DACT)
            {
                if (s->is_status_read)
                {
                    //while hard reset
                    s->status_register &= ~BL_STAT_DACT;
                    s->status_register |= BL_STAT_HARDY;
                }
                else
                {
                    s->is_status_read = true;
                }
            }

            result = s->status_register;
            break;
        }
        case BL_DAT_IN_RREG:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "%s BL_DAT_IN_RREG size= %i s->data_in_register = 0x%02x\n",__FUNCTION__,size, s->data_in_register);

            if (s->fUseLocalRam)
            {
                s->data_in_register = s->localRam.u8View[s->current_reply_byte];

            }
            else
            {
                s->data_in_register = s->reply_buffer[s->current_reply_byte];
            }

            if (s->reply_bytes_left)
            {

                s->current_reply_byte++;
                s->reply_bytes_left--;

                if (!s->reply_bytes_left)
                {
                    /*
                     * Reply finished, set command complete bit, unset data-in ready bit and
                     * interrupt the guest if enabled.
                     */
                    buslogic_command_complete(s, false);
                }
            }

            qemu_log_mask(LOG_GUEST_ERROR, "%s read BL_DAT_IN_RREG  data = 0x%02x\n",__FUNCTION__, s->data_in_register);
            result = s->data_in_register;
            break;
        }
        case BL_INT_RREG:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "%s BL_INT_RREG value = 0x%02X\n",__FUNCTION__,s->interrupt_register);
            result = s->interrupt_register;
            break;
        }



        case BL_GEO_RREG:
        {
            // qemu_log_mask(LOG_GUEST_ERROR, "%s BL_GEO_RREG\n",__FUNCTION__);
            result = s->geo_register;
            break;
        }
    }
    return result;
}


static const MemoryRegionOps bl_port_ops = {
    .read = buslogic_port_read,
    .write = buslogic_port_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

/* Callback to indicate that the SCSI layer has completed a transfer.  */
static void buslogic_scsi_transfer_data(SCSIRequest *req, uint32_t len)
{
    BuslogicState *pBusLogic = (BuslogicState *) req->hba_private;
    BUSLOGICTASKSTATE *pTaskState = pBusLogic->current_task;
    if (len == 0)
        return;


    SCSIDiskReq *r = DO_UPCAST(SCSIDiskReq, req, req);
    uint8_t *buffer = r->iov.iov_base;
    uint8_t i = 0;
    uint32_t offset = 0;
    for (i = 0; i < pTaskState->sglist->nsg; i++)
    {
            cpu_physical_memory_write(pTaskState->sglist->sg[i].base, buffer + offset, pTaskState->sglist->sg[i].len);
            offset += pTaskState->sglist->sg[i].len;
    }

    // pTaskState->TargetDevice->cOutstandingRequests--;

    //             buslogicSendIncomingMailbox(pBusLogic, pTaskState,
    //                                         BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_CMD_COMPLETED,
    //                                         BUSLOGIC_MAILBOX_INCOMING_DEVICE_STATUS_OPERATION_GOOD,
    //                                         BUSLOGIC_MAILBOX_INCOMING_COMPLETION_WITHOUT_ERROR);


}


/* Callback to indicate that the SCSI layer has completed a command.  */
static void buslogic_scsi_command_complete(SCSIRequest *req, uint32_t status, size_t resid)
{
    BuslogicState *pBusLogic = (BuslogicState *) req->hba_private;
    BUSLOGICTASKSTATE *pTaskState = pBusLogic->current_task;


    // int rc;
    // PBUSLOGICTASKSTATE pTaskState = (PBUSLOGICTASKSTATE)pSCSIRequest->pvUser;
    // PBUSLOGICDEVICE pBusLogicDevice = pTaskState->CTX_SUFF(pTargetDevice);
    // PBUSLOGIC pBusLogic = pBusLogicDevice->CTX_SUFF(pBusLogic);

    // ASMAtomicDecU32(&pBusLogicDevice->cOutstandingRequests);
    pTaskState->TargetDevice->cOutstandingRequests--;

    // if (fRedo)
    // {
    //     if (!pTaskState->fBIOS)
    //     {
    //         buslogicR3DataBufferFree(pTaskState);

    //         if (pTaskState->pbSenseBuffer)
    //             buslogicR3SenseBufferFree(pTaskState, false /* fCopy */);
    //     }

    //     /* Add to the list. */
    //     do
    //     {
    //         pTaskState->pRedoNext = ASMAtomicReadPtrT(&pBusLogic->pTasksRedoHead, PBUSLOGICTASKSTATE);
    //     } while (!ASMAtomicCmpXchgPtr(&pBusLogic->pTasksRedoHead, pTaskState, pTaskState->pRedoNext));

    //     /* Suspend the VM if not done already. */
    //     if (!ASMAtomicXchgBool(&pBusLogic->fRedo, true))
    //         buslogicR3RedoSetWarning(pBusLogic, rcReq);
    // }
    // else
    // {
    //     if (pTaskState->fBIOS)
    //     {
    //         rc = vboxscsiRequestFinished(&pBusLogic->VBoxSCSI, pSCSIRequest, rcCompletion);
    //         AssertMsgRC(rc, ("Finishing BIOS SCSI request failed rc=%Rrc\n", rc));
    //     }
    //     else
    //     {
           // buslogicDataBufferFree(pTaskState);

            // if (pTaskState->pbSenseBuffer)
            //     buslogicR3SenseBufferFree(pTaskState, (rcCompletion != SCSI_STATUS_OK));

            if (status == SCSI_STATUS_OK)
                buslogicSendIncomingMailbox(pBusLogic, pTaskState,
                                            BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_CMD_COMPLETED,
                                            BUSLOGIC_MAILBOX_INCOMING_DEVICE_STATUS_OPERATION_GOOD,
                                            BUSLOGIC_MAILBOX_INCOMING_COMPLETION_WITHOUT_ERROR);

            else if (status == SCSI_STATUS_CHECK_CONDITION)
                buslogicSendIncomingMailbox(pBusLogic, pTaskState,
                                            BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_CMD_COMPLETED,
                                            BUSLOGIC_MAILBOX_INCOMING_DEVICE_STATUS_CHECK_CONDITION,
                                            BUSLOGIC_MAILBOX_INCOMING_COMPLETION_WITH_ERROR);
            else
                qemu_log_mask(LOG_GUEST_ERROR, "%s invalid completion status\n",__FUNCTION__);
        // }
        /* Remove task from the cache. */
        // RTMemCacheFree(pBusLogic->hTaskCache, pTaskState);
    // }

    // if (pTaskState->TargetDevice->cOutstandingRequests == 0 && pBusLogic->fSignalIdle)
        // PDMDevHlpAsyncNotificationCompleted(pBusLogic->pDevInsR3);

    // return VINF_SUCCESS;
}


static void buslogic_scsi_request_cancelled(SCSIRequest *req)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s!!!!!!!!!!! function without implementation\n",__FUNCTION__);
}

//static uint32_t buslogic_get_sglist_buffer_size()
//{
//    uint32_t size = 0;
//    return size;
//}
//


/**
 * Allocate data buffer.
 *
 * @param   pTaskState    Pointer to the task state.
 * @param   GCSGList      Guest physical address of S/G list.
 * @param   cEntries      Number of list entries to read.
 * @param   pSGEList      Pointer to 32-bit S/G list storage.
 */
static void buslogicReadSGEntries(BUSLOGICTASKSTATE *pTaskState, uint64_t GCSGList, uint32_t cEntries, SGE32 *pSGEList)
{
    // PPDMDEVINS  pDevIns = pTaskState->CTX_SUFF(pTargetDevice)->CTX_SUFF(pBusLogic)->CTX_SUFF(pDevIns);
    SGE24       aSGE24[32];
    // Assert(cEntries <= RT_ELEMENTS(aSGE24));

    /* Read the S/G entries. Convert 24-bit entries to 32-bit format. */
    if (pTaskState->fIs24Bit)
    {
        // Log2(("Converting %u 24-bit S/G entries to 32-bit\n", cEntries));
        cpu_physical_memory_read(GCSGList, &aSGE24, cEntries * sizeof(SGE24));
        for (uint32_t i = 0; i < cEntries; ++i)
        {
            pSGEList[i].cbSegment              = LEN_TO_U32(aSGE24[i].acbSegment);
            pSGEList[i].u32PhysAddrSegmentBase = ADDR_TO_U32(aSGE24[i].aPhysAddrSegmentBase);
        }
    }
    else
        cpu_physical_memory_read(GCSGList, pSGEList, cEntries * sizeof(SGE32));
}


QEMUSGList *buslogic_scsi_get_sglist(SCSIRequest *req);
QEMUSGList *buslogic_scsi_get_sglist(SCSIRequest *req)
{
    QEMUSGList *sglist = NULL;
    BuslogicState *s = (BuslogicState *) req->hba_private;
    BUSLOGICTASKSTATE *pTaskState = s->current_task;

    uint32_t   cbDataCCB;
    uint32_t   u32PhysAddrCCB;
    uint8_t    ScatterGatherReadGCSize = 32;

    /* Extract the data length and physical address from the CCB. */
    if (pTaskState->fIs24Bit)
    {
        u32PhysAddrCCB  = ADDR_TO_U32(pTaskState->CommandControlBlockGuest.o.aPhysAddrData);
        cbDataCCB       = LEN_TO_U32(pTaskState->CommandControlBlockGuest.o.acbData);
    }
    else
    {
        u32PhysAddrCCB  = pTaskState->CommandControlBlockGuest.n.u32PhysAddrData;
        cbDataCCB       = pTaskState->CommandControlBlockGuest.n.cbData;
    }


    if (   (pTaskState->CommandControlBlockGuest.c.uDataDirection != BUSLOGIC_CCB_DIRECTION_NO_DATA)
        && cbDataCCB)
    {

        if (   (pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_SCATTER_GATHER)
            || (pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_RESIDUAL_SCATTER_GATHER))
        {
            //
            uint32_t cScatterGatherGCRead;
            uint32_t iScatterGatherEntry;
            SGE32    aScatterGatherReadGC[ScatterGatherReadGCSize]; /* A buffer for scatter gather list entries read from guest memory. */
            uint32_t cScatterGatherGCLeft = cbDataCCB / pTaskState->cbSGEntry;
            uint64_t GCPhysAddrScatterGatherCurrent = u32PhysAddrCCB;
            size_t cbDataToTransfer = 0;


            sglist = g_malloc0(sizeof(QEMUSGList));
            pci_dma_sglist_init(sglist, PCI_DEVICE(req->hba_private), cScatterGatherGCLeft);
            if (!sglist)
            {
                // return because sglist can't be allocated
                return 0;
            }


            /* Count number of bytes to transfer. */
            do
            {
                cScatterGatherGCRead =   (cScatterGatherGCLeft < ScatterGatherReadGCSize)
                                        ? cScatterGatherGCLeft
                                        : ScatterGatherReadGCSize;
                cScatterGatherGCLeft -= cScatterGatherGCRead;

                buslogicReadSGEntries(pTaskState, GCPhysAddrScatterGatherCurrent, cScatterGatherGCRead, aScatterGatherReadGC);

                for (iScatterGatherEntry = 0; iScatterGatherEntry < cScatterGatherGCRead; iScatterGatherEntry++)
                {

                    //uint64_t    GCPhysAddrDataBase;
                    //GCPhysAddrDataBase = (uint64_t)aScatterGatherReadGC[iScatterGatherEntry].u32PhysAddrSegmentBase;
                    cbDataToTransfer += aScatterGatherReadGC[iScatterGatherEntry].cbSegment;
                    qemu_sglist_add(sglist, aScatterGatherReadGC[iScatterGatherEntry].u32PhysAddrSegmentBase, aScatterGatherReadGC[iScatterGatherEntry].cbSegment);

                }

                /* Set address to the next entries to read. */
                GCPhysAddrScatterGatherCurrent += cScatterGatherGCRead * pTaskState->cbSGEntry;
            } while (cScatterGatherGCLeft > 0);
        }
        else if (   pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB
                 || pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_RESIDUAL_DATA_LENGTH)
        {
            /* The buffer is not scattered. */
            uint64_t GCPhysAddrDataBase = u32PhysAddrCCB;

            sglist = g_malloc0(sizeof(QEMUSGList));
            pci_dma_sglist_init(sglist, PCI_DEVICE(req->hba_private), 1);
            if (!sglist)
            {
                // return because sglist can't be allocated
                return 0;
            }
            qemu_sglist_add(sglist, GCPhysAddrDataBase, pTaskState->sglEntry.len);

            // pTaskState->sglEntry.len = cbDataCCB;
            // pTaskState->dataBuffer = g_malloc0(pTaskState->sglEntry.len * sizeof(uint8_t));
            // if (!pTaskState->dataBuffer)
            // {
            //     return -1;
            // }
            // pTaskState->sglEntry.base = pTaskState->dataBuffer;

            // /* Copy the data into the buffer. */
            // cpu_physical_memory_read(GCPhysAddrDataBase, pTaskState->dataBuffer, pTaskState->sglEntry.len);
        }
    }


    // sglist = g_malloc0(sizeof(QEMUSGList));
    // pci_dma_sglist_init(sglist, PCI_DEVICE(req->hba_private), 1);
    // if (!sglist)
    // {
    //     // return because sglist can't be allocated
    //     return 0;
    // }
    pTaskState->sglist = sglist;
    // qemu_sglist_add(sglist, pTaskState->sglEntry.base, pTaskState->sglEntry.len);
    return sglist;
}


static const struct SCSIBusInfo bl_scsi_info = {
    .tcq = true,
    .max_target = BUSLOGIC_MAX_DEVICES,
    .max_lun = 0,  /* LUN support is buggy */

    .transfer_data = buslogic_scsi_transfer_data,
    .complete = buslogic_scsi_command_complete,
    .cancel = buslogic_scsi_request_cancelled,
    .get_sg_list = buslogic_scsi_get_sglist
};


static void bl_ram_write(void *opaque, hwaddr addr,
                          uint64_t val, unsigned size)
{

    qemu_log_mask(LOG_GUEST_ERROR, "%s  without implementation\n",__FUNCTION__);
}

static uint64_t bl_ram_read(void *opaque, hwaddr addr,
                             unsigned size)
{

    qemu_log_mask(LOG_GUEST_ERROR, "%s without implementation\n",__FUNCTION__);
    return 0;
}

static const MemoryRegionOps bl_ram_ops = {
    .read = bl_ram_read,
    .write = bl_ram_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static uint64_t bl_mmio_read(void *opaque, hwaddr addr,
                            unsigned size)
{
    // LSIState *s = opaque;
    // return lsi_reg_readb(s, addr & 0xff);
    qemu_log_mask(LOG_GUEST_ERROR, "%s  without implementation\n",__FUNCTION__);
    return 1;
}

static void bl_mmio_write(void *opaque, hwaddr addr,
                         uint64_t val, unsigned size)
{
    // LSIState *s = opaque;
    // lsi_reg_writeb(s, addr & 0xff, val);
    qemu_log_mask(LOG_GUEST_ERROR, "%s  without implementation\n",__FUNCTION__);
}

static const MemoryRegionOps bl_mmio_ops = {
    .read = bl_mmio_read,
    .write = bl_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 8,
        .max_access_size = 8,
    },
};







static void buslogic_scsi_realize(PCIDevice *dev, Error **errp)
{
    BuslogicState *s = BUSLOGIC_BT958(dev);
    DeviceState *d = DEVICE(dev);
    BusChild *kid;
    memset(s->deviceStates, 0, sizeof(BUSLOGICDEVICE) * BUSLOGIC_MAX_DEVICES);

    // dev->bus_master_as
    // qemu_log_mask(LOG_GUEST_ERROR, "%p\n", pci_get_address_space(PCI_DEVICE(s)));
    uint8_t *pci_conf;
    qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);

    pci_conf = dev->config;

    /* PCI latency timer = 255 */
    pci_conf[PCI_LATENCY_TIMER] = 0x0;
    /* Interrupt pin A */
    pci_conf[PCI_INTERRUPT_PIN] = 0x01;
    // pci_conf[PCI_INTERRUPT_LINE] = 0x11;


    memory_region_init_io(&s->mmio_io, OBJECT(s), &bl_mmio_ops, s,
                          "buslogic-mmio", 0x4000);
    memory_region_init_io(&s->port_io, OBJECT(s), &bl_port_ops, s,
                          "BusLogic-io", 256);
    memory_region_init_io(&s->ram_io, OBJECT(s), &bl_ram_ops, s,
                          "buslogic-queue", 0x40000);


    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->port_io);
    pci_register_bar(dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64, &s->mmio_io);

    pci_register_bar(dev, 3, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->ram_io);

    QTAILQ_INIT(&s->queue);

    scsi_bus_new(&s->bus, sizeof(s->bus), d, &bl_scsi_info, NULL);
    if (!d->hotplugged) {
        scsi_bus_legacy_handle_cmdline(&s->bus, errp);
    }


    //fill arr of devices
    QTAILQ_FOREACH_REVERSE(kid, &s->bus.qbus.children, ChildrenHead, sibling) {
        DeviceState *qdev = kid->child;
        SCSIDevice *dev = SCSI_DEVICE(qdev);
        s->deviceStates[dev->lun].fPresent = true;
        s->deviceStates[dev->lun].iLUN = dev->lun;
        s->deviceStates[dev->lun].channel = dev->channel;
    }

    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    s->mailboxCheckTimer = timer_new_ns(QEMU_CLOCK_REALTIME, buslogic_check_mailboxes, s);
    timer_mod_ns(s->mailboxCheckTimer, now + 10000);
}

/**
 * Advances the mailbox pointer to the next slot.
 */
void buslogicOutgoingMailboxAdvance(BuslogicState *s);
void buslogicOutgoingMailboxAdvance(BuslogicState *s)
{
    s->uMailboxOutgoingPositionCurrent = (s->uMailboxOutgoingPositionCurrent + 1) % s->mailbox;
    qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);
}


/**
 * Read a mailbox from guest memory. Convert 24-bit mailboxes to
 * 32-bit format.
 *
 * @returns Mailbox guest physical address.
 * @param   pBusLogic    Pointer to the BusLogic instance data.
 * @param   pTaskStat    Pointer to the task state being set up.
 */
 uint64_t buslogicReadOutgoingMailbox(BuslogicState *s, BUSLOGICTASKSTATE *TaskState);
uint64_t buslogicReadOutgoingMailbox(BuslogicState *s, BUSLOGICTASKSTATE *TaskState)
{
    uint64_t    GCMailbox;
    Mailbox24   Mbx24;
    Mbx24.uCmdState = 0;

    if (s->fMbxIs24Bit)
    {
        GCMailbox = s->GCPhysAddrMailboxOutgoingBase + (s->uMailboxOutgoingPositionCurrent * sizeof(Mailbox24));
        // PDMDevHlpPhysRead(s->CTX_SUFF(pDevIns), GCMailbox, &Mbx24, sizeof(Mailbox24));
        cpu_physical_memory_read(GCMailbox, &Mbx24, sizeof(Mailbox24));
        TaskState->MailboxGuest.u32PhysAddrCCB    = ADDR_TO_U32(Mbx24.aPhysAddrCCB);
        TaskState->MailboxGuest.u.out.uActionCode = Mbx24.uCmdState;
    }
    else
    {
        GCMailbox = s->GCPhysAddrMailboxOutgoingBase + (s->uMailboxOutgoingPositionCurrent * sizeof(Mailbox32));
        cpu_physical_memory_read(GCMailbox, &TaskState->MailboxGuest, sizeof(Mailbox32));
    }

    return GCMailbox;
}


/** Convert sense buffer length taking into account shortcut values. */
//static uint32_t buslogicConvertSenseBufferLength(uint32_t cbSense)
//{
//    /* Convert special sense buffer length values. */
//    if (cbSense == 0)
//    {
//        cbSense = 14;   /* 0 means standard 14-byte buffer. */
//    }
//    else if (cbSense == 1)
//    {
//        cbSense = 0;    /* 1 means no sense data. */
//    }
//    else if (cbSense < 8)
//    {
//        // AssertMsgFailed(("Reserved cbSense value of %d used!\n", cbSense));
//    }
//
//    return cbSense;
//}
//


/**
 * Alloc the sense buffer.
 *
 * @returns VBox status code.
 * @param   pTaskState    Pointer to the task state.
 * @note Current assumption is that the sense buffer is not scattered and does not cross a page boundary.
 */
//static int buslogicSenseBufferAlloc(BUSLOGICTASKSTATE *pTaskState)
//{
//    // PDMDEVINS *pDevIns = pTaskState->TargetDevice->pBusLogic->pDevIns;
//    uint32_t   cbSenseBuffer;
//
//    pTaskState->pbSenseBuffer = NULL;
//
//    cbSenseBuffer = buslogicConvertSenseBufferLength(pTaskState->CommandControlBlockGuest.c.cbSenseData);
//    if (cbSenseBuffer)
//    {
//        pTaskState->pbSenseBuffer = (uint8_t *)g_malloc0(cbSenseBuffer);
//        if (!pTaskState->pbSenseBuffer)
//        {
//            //no memory
//            return -1;
//        }
//        // memset(pTaskState->pbSenseBuffer, 0, cbSenseBuffer);
//    }
//
//    return 1;
//}
//





/**
 * Allocate data buffer.
 *
 * @returns VBox status code.
 * @param   pTaskState    Pointer to the task state.
 */
//static int buslogicDataBufferAlloc(BUSLOGICTASKSTATE *pTaskState)
//{
    // // PDMDEVINS *pDevIns = pTaskState->CTX_SUFF(pTargetDevice)->CTX_SUFF(pBusLogic)->CTX_SUFF(pDevIns);
    // uint32_t   cbDataCCB;
    // uint32_t   u32PhysAddrCCB;
    // uint8_t    ScatterGatherReadGCSize = 32;

    // /* Extract the data length and physical address from the CCB. */
    // if (pTaskState->fIs24Bit)
    // {
    //     u32PhysAddrCCB  = ADDR_TO_U32(pTaskState->CommandControlBlockGuest.o.aPhysAddrData);
    //     cbDataCCB       = LEN_TO_U32(pTaskState->CommandControlBlockGuest.o.acbData);
    // }
    // else
    // {
    //     u32PhysAddrCCB  = pTaskState->CommandControlBlockGuest.n.u32PhysAddrData;
    //     cbDataCCB       = pTaskState->CommandControlBlockGuest.n.cbData;
    // }


    // if (   (pTaskState->CommandControlBlockGuest.c.uDataDirection != BUSLOGIC_CCB_DIRECTION_NO_DATA)
    //     && cbDataCCB)
    // {

    //     if (   (pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_SCATTER_GATHER)
    //         || (pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_RESIDUAL_SCATTER_GATHER))
    //     {
    //         //
    //         uint32_t cScatterGatherGCRead;
    //         uint32_t iScatterGatherEntry;
    //         SGE32    aScatterGatherReadGC[ScatterGatherReadGCSize]; /* A buffer for scatter gather list entries read from guest memory. */
    //         uint32_t cScatterGatherGCLeft = cbDataCCB / pTaskState->cbSGEntry;
    //         uint64_t GCPhysAddrScatterGatherCurrent = u32PhysAddrCCB;
    //         size_t cbDataToTransfer = 0;

    //         /* Count number of bytes to transfer. */
    //         do
    //         {
    //             cScatterGatherGCRead =   (cScatterGatherGCLeft < ScatterGatherReadGCSize)
    //                                     ? cScatterGatherGCLeft
    //                                     : ScatterGatherReadGCSize;
    //             cScatterGatherGCLeft -= cScatterGatherGCRead;

    //             buslogicReadSGEntries(pTaskState, GCPhysAddrScatterGatherCurrent, cScatterGatherGCRead, aScatterGatherReadGC);

    //             for (iScatterGatherEntry = 0; iScatterGatherEntry < cScatterGatherGCRead; iScatterGatherEntry++)
    //             {
    //                 // uint64_t    GCPhysAddrDataBase;

    //                 // GCPhysAddrDataBase = (uint64_t)aScatterGatherReadGC[iScatterGatherEntry].u32PhysAddrSegmentBase;
    //                 cbDataToTransfer += aScatterGatherReadGC[iScatterGatherEntry].cbSegment;


    //             }

    //             /* Set address to the next entries to read. */
    //             GCPhysAddrScatterGatherCurrent += cScatterGatherGCRead * pTaskState->cbSGEntry;
    //         } while (cScatterGatherGCLeft > 0);


    //         /* Allocate buffer */
    //         pTaskState->DataSeg.cbSeg = cbDataToTransfer;
    //         pTaskState->DataSeg.pvSeg = g_malloc0(pTaskState->DataSeg.cbSeg);
    //         if (!pTaskState->DataSeg.pvSeg)
    //             return -1;

    //         /* Copy the data if needed */
    //         if (   (pTaskState->CommandControlBlockGuest.c.uDataDirection == BUSLOGIC_CCB_DIRECTION_OUT)
    //             || (pTaskState->CommandControlBlockGuest.c.uDataDirection == BUSLOGIC_CCB_DIRECTION_UNKNOWN))
    //         {
    //             cScatterGatherGCLeft = cbDataCCB / pTaskState->cbSGEntry;
    //             GCPhysAddrScatterGatherCurrent = u32PhysAddrCCB;
    //             uint8_t *pbData = (uint8_t *)pTaskState->DataSeg.pvSeg;

    //             do
    //             {
    //                 cScatterGatherGCRead =   (cScatterGatherGCLeft < ScatterGatherReadGCSize)
    //                                         ? cScatterGatherGCLeft
    //                                         : ScatterGatherReadGCSize;
    //                 cScatterGatherGCLeft -= cScatterGatherGCRead;

    //                 buslogicReadSGEntries(pTaskState, GCPhysAddrScatterGatherCurrent, cScatterGatherGCRead, aScatterGatherReadGC);

    //                 for (iScatterGatherEntry = 0; iScatterGatherEntry < cScatterGatherGCRead; iScatterGatherEntry++)
    //                 {
    //                     uint64_t    GCPhysAddrDataBase;

    //                     // Log(("%s: iScatterGatherEntry=%u\n", __FUNCTION__, iScatterGatherEntry));

    //                     GCPhysAddrDataBase = (uint64_t)aScatterGatherReadGC[iScatterGatherEntry].u32PhysAddrSegmentBase;
    //                     cbDataToTransfer = aScatterGatherReadGC[iScatterGatherEntry].cbSegment;


    //                     cpu_physical_memory_read(GCPhysAddrDataBase, pbData, cbDataToTransfer);
    //                     pbData += cbDataToTransfer;
    //                 }

    //                 /* Set address to the next entries to read. */
    //                 GCPhysAddrScatterGatherCurrent += cScatterGatherGCRead * pTaskState->cbSGEntry;
    //             } while (cScatterGatherGCLeft > 0);
    //         }

    //     }
    //     else if (   pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB
    //              || pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_RESIDUAL_DATA_LENGTH)
    //     {
    //         /* The buffer is not scattered. */
    //         uint64_t GCPhysAddrDataBase = u32PhysAddrCCB;


    //         pTaskState->DataSeg.cbSeg = cbDataCCB;
    //         pTaskState->DataSeg.pvSeg = g_malloc0(pTaskState->DataSeg.cbSeg);
    //         if (!pTaskState->DataSeg.pvSeg)
    //             return -1;

    //         /* Copy the data into the buffer. */
    //         cpu_physical_memory_read(GCPhysAddrDataBase, pTaskState->DataSeg.pvSeg, pTaskState->DataSeg.cbSeg);
    //     }
    // }

//    return 1;
//}







/**
 * Free allocated resources used for the scatter gather list.
 *
 * @returns nothing.
 * @param   pTaskState    Pointer to the task state.
 */
void buslogicDataBufferFree(BUSLOGICTASKSTATE *pTaskState)
{
    // PPDMDEVINS pDevIns = pTaskState->CTX_SUFF(pTargetDevice)->CTX_SUFF(pBusLogic)->CTX_SUFF(pDevIns);
    uint32_t   cbDataCCB;
    uint32_t   u32PhysAddrCCB;
    uint8_t ScatterGatherReadGCSize = 32;

    /* Extract the data length and physical address from the CCB. */
    if (pTaskState->fIs24Bit)
    {
        u32PhysAddrCCB  = ADDR_TO_U32(pTaskState->CommandControlBlockGuest.o.aPhysAddrData);
        cbDataCCB       = LEN_TO_U32(pTaskState->CommandControlBlockGuest.o.acbData);
    }
    else
    {
        u32PhysAddrCCB  = pTaskState->CommandControlBlockGuest.n.u32PhysAddrData;
        cbDataCCB       = pTaskState->CommandControlBlockGuest.n.cbData;
    }

#if 1
    /* Hack for NT 10/91: A CCB describes a 2K buffer, but TEST UNIT READY is executed. This command
     * returns no data, hence the buffer must be left alone!
     */
    if (pTaskState->CommandControlBlockGuest.c.abCDB[0] == 0)
        cbDataCCB = 0;
#endif

    // LogFlowFunc(("pTaskState=%#p cbDataCCB=%u direction=%u cbSeg=%u\n", pTaskState, cbDataCCB,
                 // pTaskState->CommandControlBlockGuest.c.uDataDirection, pTaskState->DataSeg.cbSeg));

    if (   (cbDataCCB > 0)
        && (   (pTaskState->CommandControlBlockGuest.c.uDataDirection == BUSLOGIC_CCB_DIRECTION_IN)
            || (pTaskState->CommandControlBlockGuest.c.uDataDirection == BUSLOGIC_CCB_DIRECTION_UNKNOWN)))
    {
        if (   (pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_SCATTER_GATHER)
            || (pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_RESIDUAL_SCATTER_GATHER))
        {
            uint32_t cScatterGatherGCRead;
            uint32_t iScatterGatherEntry;
            SGE32    aScatterGatherReadGC[ScatterGatherReadGCSize]; /* Number of scatter gather list entries read from guest memory. */
            uint32_t cScatterGatherGCLeft = cbDataCCB / pTaskState->cbSGEntry;
            uint64_t GCPhysAddrScatterGatherCurrent = u32PhysAddrCCB;
            uint8_t *pbData = (uint8_t *)pTaskState->sglEntry.base;

            do
            {
                cScatterGatherGCRead = (cScatterGatherGCLeft < ScatterGatherReadGCSize)
                                     ? cScatterGatherGCLeft
                                     : ScatterGatherReadGCSize;
                cScatterGatherGCLeft -= cScatterGatherGCRead;

                buslogicReadSGEntries(pTaskState, GCPhysAddrScatterGatherCurrent, cScatterGatherGCRead, aScatterGatherReadGC);

                for (iScatterGatherEntry = 0; iScatterGatherEntry < cScatterGatherGCRead; iScatterGatherEntry++)
                {
                    uint64_t    GCPhysAddrDataBase;
                    size_t      cbDataToTransfer;

                    // Log(("%s: iScatterGatherEntry=%u\n", __FUNCTION__, iScatterGatherEntry));

                    GCPhysAddrDataBase = (uint64_t)aScatterGatherReadGC[iScatterGatherEntry].u32PhysAddrSegmentBase;
                    cbDataToTransfer = aScatterGatherReadGC[iScatterGatherEntry].cbSegment;

                    // Log(("%s: GCPhysAddrDataBase=%RGp cbDataToTransfer=%u\n", __FUNCTION__, GCPhysAddrDataBase, cbDataToTransfer));

                    cpu_physical_memory_write(GCPhysAddrDataBase, pbData, cbDataToTransfer);
                    pbData += cbDataToTransfer;
                }

                /* Set address to the next entries to read. */
                GCPhysAddrScatterGatherCurrent += cScatterGatherGCRead * pTaskState->cbSGEntry;
            } while (cScatterGatherGCLeft > 0);

        }
        else if (   pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB
                 || pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_RESIDUAL_DATA_LENGTH)
        {
            /* The buffer is not scattered. */
            uint64_t GCPhysAddrDataBase = u32PhysAddrCCB;

            // AssertMsg(GCPhysAddrDataBase != 0, ("Physical address is 0\n"));

            // Log(("Non-scattered buffer:\n"));
            // Log(("u32PhysAddrData=%#x\n", u32PhysAddrCCB));
            // Log(("cbData=%u\n", cbDataCCB));
            // Log(("GCPhysAddrDataBase=0x%RGp\n", GCPhysAddrDataBase));

            /* Copy the data into the guest memory. */
            cpu_physical_memory_write(GCPhysAddrDataBase, (void*)pTaskState->sglEntry.base, pTaskState->sglEntry.len);
        }

    }
    /* Update residual data length. */
    if (   (pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_RESIDUAL_DATA_LENGTH)
        || (pTaskState->CommandControlBlockGuest.c.uOpcode == BUSLOGIC_CCB_OPCODE_INITIATOR_CCB_RESIDUAL_SCATTER_GATHER))
    {
        uint32_t    cbResidual;

        /** @todo we need to get the actual transfer length from the VSCSI layer?! */
        cbResidual = 0; //LEN_TO_U32(pTaskState->CCBGuest.acbData) - ???;
        if (pTaskState->fIs24Bit)
            U32_TO_LEN(pTaskState->CommandControlBlockGuest.o.acbData, cbResidual);
        else
            pTaskState->CommandControlBlockGuest.n.cbData = cbResidual;
    }

    if (pTaskState->dataBuffer)
    {
        g_free(pTaskState->dataBuffer);
        pTaskState->dataBuffer = NULL;
    }

    pTaskState->sglEntry.base = 0;
    pTaskState->sglEntry.len = 0;

    if (pTaskState->sglist)
    {
        qemu_sglist_destroy(pTaskState->sglist);
        g_free(pTaskState->sglist);
        pTaskState->sglist = NULL;
    }

}




/**
 * Free the sense buffer.
 *
 * @returns nothing.
 * @param   pTaskState   Pointer to the task state.
 * @param   fCopy        If sense data should be copied to guest memory.
 */
// static void buslogicSenseBufferFree(BUSLOGICTASKSTATE *pTaskState, bool fCopy)
// {
//     uint32_t    cbSenseBuffer;

//     cbSenseBuffer = buslogicConvertSenseBufferLength(pTaskState->CommandControlBlockGuest.c.cbSenseData);

//     /* Copy the sense buffer into guest memory if requested. */
//     if (fCopy && cbSenseBuffer)
//     {
//         // PPDMDEVINS  pDevIns = pTaskState->CTX_SUFF(pTargetDevice)->CTX_SUFF(pBusLogic)->CTX_SUFF(pDevIns);
//         uint64_t    GCPhysAddrSenseBuffer;

//          With 32-bit CCBs, the (optional) sense buffer physical address is provided separately.
//          * On the other hand, with 24-bit CCBs, the sense buffer is simply located at the end of
//          * the CCB, right after the variable-length CDB.

//         if (pTaskState->fIs24Bit)
//         {
//             GCPhysAddrSenseBuffer  = pTaskState->MailboxGuest.u32PhysAddrCCB;
//             GCPhysAddrSenseBuffer += pTaskState->CommandControlBlockGuest.c.cbCDB + RT_OFFSETOF(CCB24, abCDB);
//         }
//         else
//             GCPhysAddrSenseBuffer = pTaskState->CommandControlBlockGuest.n.u32PhysAddrSenseData;

//         // Log3(("%s: sense buffer: %.*Rhxs\n", __FUNCTION__, cbSenseBuffer, pTaskState->pbSenseBuffer));
//         cpu_physical_memory_write(GCPhysAddrSenseBuffer, pTaskState->pbSenseBuffer, cbSenseBuffer);
//     }

//     g_free(pTaskState->pbSenseBuffer);
//     pTaskState->pbSenseBuffer = NULL;
// }


/**
 * Send a mailbox with set status codes to the guest.
 *
 * @returns nothing.
 * @param   pBusLogic                 Pointer to the BusLogic device instance.
 * @param   pTaskState                Pointer to the task state with the mailbox to send.
 * @param   uHostAdapterStatus        The host adapter status code to set.
 * @param   uDeviceStatus             The target device status to set.
 * @param   uMailboxCompletionCode    Completion status code to set in the mailbox.
 */
void buslogicSendIncomingMailbox(BuslogicState *pBusLogic, BUSLOGICTASKSTATE *pTaskState,
                                          uint8_t uHostAdapterStatus, uint8_t uDeviceStatus,
                                          uint8_t uMailboxCompletionCode)
{
    PCIDevice *d = PCI_DEVICE(pBusLogic);
    qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);
    pTaskState->MailboxGuest.u.in.uHostAdapterStatus = uHostAdapterStatus;
    pTaskState->MailboxGuest.u.in.uTargetDeviceStatus = uDeviceStatus;
    pTaskState->MailboxGuest.u.in.uCompletionCode = uMailboxCompletionCode;

    // int rc = PDMCritSectEnter(&pBusLogic->CritSectIntr, VINF_SUCCESS);
    // AssertRC(rc);

    uint64_t GCPhysAddrMailboxIncoming = pBusLogic->GCPhysAddrMailboxIncomingBase
                                       + (   pBusLogic->uMailboxIncomingPositionCurrent
                                          * (pTaskState->fIs24Bit ? sizeof(Mailbox24) : sizeof(Mailbox32)) );

    if (uMailboxCompletionCode != BUSLOGIC_MAILBOX_INCOMING_COMPLETION_ABORTED_NOT_FOUND)
    {
        uint64_t GCPhysAddrCCB = pTaskState->MailboxGuest.u32PhysAddrCCB;

        /* Update CCB. */
        pTaskState->CommandControlBlockGuest.c.uHostAdapterStatus = uHostAdapterStatus;
        pTaskState->CommandControlBlockGuest.c.uDeviceStatus      = uDeviceStatus;

        /* Rewrite CCB up to the CDB; perhaps more than necessary. */
        cpu_physical_memory_write(GCPhysAddrCCB,
                              &pTaskState->CommandControlBlockGuest, RT_OFFSETOF(CCBC, abCDB));
    }

// # ifdef RT_STRICT
//     uint8_t     uCode;
//     unsigned    uCodeOffs = pTaskState->fIs24Bit ? RT_OFFSETOF(Mailbox24, uCmdState) : RT_OFFSETOF(Mailbox32, u.out.uActionCode);
//     PDMDevHlpPhysRead(pBusLogic->CTX_SUFF(pDevIns), GCPhysAddrMailboxIncoming + uCodeOffs, &uCode, sizeof(uCode));
//     Assert(uCode == BUSLOGIC_MAILBOX_INCOMING_COMPLETION_FREE);
// # endif

    /* Update mailbox. */
    if (pTaskState->fIs24Bit)
    {
        Mailbox24   Mbx24;

        Mbx24.uCmdState = pTaskState->MailboxGuest.u.in.uCompletionCode;
        U32_TO_ADDR(Mbx24.aPhysAddrCCB, pTaskState->MailboxGuest.u32PhysAddrCCB);
        // Log(("24-bit mailbox: completion code=%u, CCB at %RGp\n", Mbx24.uCmdState, (RTGCPHYS)ADDR_TO_U32(Mbx24.aPhysAddrCCB)));
        cpu_physical_memory_write(GCPhysAddrMailboxIncoming, &Mbx24, sizeof(Mailbox24));
    }
    else
    {
        cpu_physical_memory_write(GCPhysAddrMailboxIncoming,
                              &pTaskState->MailboxGuest, sizeof(Mailbox32));
    }

    /* Advance to next mailbox position. */
    pBusLogic->uMailboxIncomingPositionCurrent++;
    if (pBusLogic->uMailboxIncomingPositionCurrent >= pBusLogic->mailbox)
        pBusLogic->uMailboxIncomingPositionCurrent = 0;

    buslogic_set_interrupt(pBusLogic, false, BL_INTR_IMBL);
    pci_set_irq(d, 1);

    // PDMCritSectLeave(&pBusLogic->CritSectIntr);
}

int buslogicDeviceSCSIRequestSetup(BuslogicState *s, BUSLOGICTASKSTATE *TaskState);
int buslogicDeviceSCSIRequestSetup(BuslogicState *s, BUSLOGICTASKSTATE *TaskState)
{
    int rc = 1;
    qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);
    uint8_t uTargetIdCCB;
    BUSLOGICDEVICE *pTargetDevice = NULL;
    SCSIDevice *pDev = NULL;
    int32_t retval;


    /* Fetch the CCB from guest memory. */
    /** @todo How much do we really have to read? */
    uint64_t GCPhysAddrCCB = (uint64_t)TaskState->MailboxGuest.u32PhysAddrCCB;
    cpu_physical_memory_read(GCPhysAddrCCB,
                        &TaskState->CommandControlBlockGuest, sizeof(CCB32));



    uTargetIdCCB = TaskState->fIs24Bit ? TaskState->CommandControlBlockGuest.o.uTargetId : TaskState->CommandControlBlockGuest.n.uTargetId;
    qemu_log_mask(LOG_GUEST_ERROR, "targetID = 0x%1x\n",uTargetIdCCB);

    //scsi_device_show(&s->bus);

    pTargetDevice = &s->deviceStates[uTargetIdCCB];
    TaskState->TargetDevice = pTargetDevice;


    /* Alloc required buffers. */
    // rc = buslogicDataBufferAlloc(TaskState);

    // rc = buslogicSenseBufferAlloc(TaskState);
    // qemu_log_mask(LOG_GUEST_ERROR, "targetID = %hi present = %i\n",uTargetIdCCB, s->deviceStates[uTargetIdCCB].fPresent);

    /* Check if device is present on bus. If not return error immediately and don't process this further. */
    if (!s->deviceStates[uTargetIdCCB].fPresent)
    {
        qemu_log_mask(LOG_GUEST_ERROR, "device not on bus\n");
        if (TaskState->dataBuffer)
        {
            buslogicDataBufferFree(TaskState);
        }

        // if (TaskState->pbSenseBuffer)
        //     buslogicSenseBufferFree(TaskState, true);

        buslogicSendIncomingMailbox(s, TaskState,
                                    BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_SCSI_SELECTION_TIMEOUT,
                                    BUSLOGIC_MAILBOX_INCOMING_DEVICE_STATUS_OPERATION_GOOD,
                                    BUSLOGIC_MAILBOX_INCOMING_COMPLETION_WITH_ERROR);

        // RTMemCacheFree(s->hTaskCache, TaskState);

    }
    else
    {
        qemu_log_mask(LOG_GUEST_ERROR, "device on bus!!! do scsi\n");
        /* Setup SCSI request. */
        // TaskState->ScsiRequest.uLogicalUnit = TaskState->fIs24Bit ? TaskState->CommandControlBlockGuest.o.uLogicalUnit :TaskState->CommandControlBlockGuest.n.uLogicalUnit;
        // if (TaskState->CommandControlBlockGuest.c.uDataDirection == BUSLOGIC_CCB_DIRECTION_UNKNOWN)
        // {
            // TaskState->ScsiRequest.uDataDirection = PDMSCSIREQUESTTXDIR_UNKNOWN;
            // TaskState->ScsiRequest.
        // }
        // else if (TaskState->CommandControlBlockGuest.c.uDataDirection == BUSLOGIC_CCB_DIRECTION_IN)
        // {
            // TaskState->ScsiRequest.uDataDirection = PDMSCSIREQUESTTXDIR_FROM_DEVICE;
        // }
        // else if (TaskState->CommandControlBlockGuest.c.uDataDirection == BUSLOGIC_CCB_DIRECTION_OUT)
        // {
            // TaskState->ScsiRequest.uDataDirection = PDMSCSIREQUESTTXDIR_TO_DEVICE;
        // }
        // else if (TaskState->CommandControlBlockGuest.c.uDataDirection == BUSLOGIC_CCB_DIRECTION_NO_DATA)
        // {
            // TaskState->ScsiRequest.uDataDirection = PDMSCSIREQUESTTXDIR_NONE;
        // }
        // else
        // {
            // qemu_log_mask(LOG_GUEST_ERROR, "Invalid data direction\n");
            // AssertMsgFailed(("Invalid data direction type %d\n", TaskState->CommandControlBlockGuest.c.uDataDirection));
        // }

        // TaskState->ScsiRequest.cbCDB                 = TaskState->CommandControlBlockGuest.c.cbCDB;
        // TaskState->ScsiRequest.pbCDB                 = TaskState->CommandControlBlockGuest.c.abCDB;
        // if (TaskState->DataSeg.cbSeg)
        // {
            // TaskState->ScsiRequest.cbScatterGather       = TaskState->DataSeg.cbSeg;
            // TaskState->ScsiRequest.cScatterGatherEntries = 1;
            // TaskState->ScsiRequest.paScatterGatherHead   = &TaskState->DataSeg;
        // }
        // else
        // {
            // TaskState->ScsiRequest.cbScatterGather       = 0;
            // TaskState->ScsiRequest.cScatterGatherEntries = 0;
            // TaskState->ScsiRequest.paScatterGatherHead   = NULL;
        // }
        // TaskState->pScsiRequest->sense_len       = buslogicConvertSenseBufferLength(TaskState->CommandControlBlockGuest.c.cbSenseData);
        // TaskState->pScsiRequest->pbSenseBuffer         = TaskState->pbSenseBuffer;
        // TaskState->ScsiRequest.pvUser                = TaskState;


        pDev = scsi_device_find(&s->bus, pTargetDevice->channel, pTargetDevice->id, pTargetDevice->iLUN);

        if (!pDev)
        {
            qemu_log_mask(LOG_GUEST_ERROR, "Can not find target device \n");
            return -1;
        }
        else
        {
            qemu_log_mask(LOG_GUEST_ERROR, "cool !!! we are find the target device\n");
        }




        // ASMAtomicIncU32(&pTargetDevice->cOutstandingRequests);
        pTargetDevice->cOutstandingRequests++;
        TaskState->pScsiRequest = scsi_req_new(pDev, /*s->current->tag*/0x20, pTargetDevice->iLUN, TaskState->CommandControlBlockGuest.c.abCDB, s);

        retval = scsi_req_enqueue(TaskState->pScsiRequest);
        // TaskState->sglist = buslogic_scsi_get_sglist(TaskState->pScsiRequest);
        if (retval)
        {
            if (retval > 0)
            {

            }
            else if (retval < 0)
            {

            }
            scsi_req_continue(TaskState->pScsiRequest);
        }
        // rc = pTargetDevice->pDrvSCSIConnector->pfnSCSIRequestSend(pTargetDevice->pDrvSCSIConnector, &TaskState->ScsiRequest);
    }

    return rc;
}

int buslogicDeviceSCSIRequestAbort(BuslogicState *s, BUSLOGICTASKSTATE *TaskState);
int buslogicDeviceSCSIRequestAbort(BuslogicState *s, BUSLOGICTASKSTATE *TaskState)
{
    int             rc = 1;
    qemu_log_mask(LOG_GUEST_ERROR, "WARNING !!!! this is empty function %s\n",__FUNCTION__);
    // uint8_t         uTargetIdCCB;
    // BUSLOGICDEVICE *pTargetDevice;
    // uint64_t        GCPhysAddrCCB = (RTGCPHYS)pTaskState->MailboxGuest.u32PhysAddrCCB;

    // PDMDevHlpPhysRead(pBusLogic->CTX_SUFF(pDevIns), GCPhysAddrCCB,
    //                   &pTaskState->CommandControlBlockGuest, sizeof(CCB32));

    // uTargetIdCCB = pTaskState->fIs24Bit ? pTaskState->CommandControlBlockGuest.o.uTargetId : pTaskState->CommandControlBlockGuest.n.uTargetId;
    // pTargetDevice = &pBusLogic->aDeviceStates[uTargetIdCCB];
    // pTaskState->CTX_SUFF(pTargetDevice) = pTargetDevice;

    // buslogicR3SendIncomingMailbox(pBusLogic, pTaskState,
    //                               BUSLOGIC_MAILBOX_INCOMING_ADAPTER_STATUS_ABORT_QUEUE_GENERATED,
    //                               BUSLOGIC_MAILBOX_INCOMING_DEVICE_STATUS_OPERATION_GOOD,
    //                               BUSLOGIC_MAILBOX_INCOMING_COMPLETION_ABORTED_NOT_FOUND);

    // RTMemCacheFree(pBusLogic->hTaskCache, pTaskState);

    return rc;
}

static int buslogicProcessMailboxNext(BuslogicState *s)
{
    s->current_task = NULL;
    // PCIDevice *pci_dev = PCI_DEVICE(s);
    // qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);
    uint64_t           GCPhysAddrMailboxCurrent = 0;
    int result = 1;

    s->current_task = (BUSLOGICTASKSTATE*) g_malloc0(sizeof(BUSLOGICTASKSTATE));
    //need to check return value
    if (!s->current_task)
    {
        printf("Error while alloc taskstate\n");
        return -1;
    }

    // now fill some fields of the taskstate struct
    s->current_task->fBIOS = false;
    s->current_task->fIs24Bit = s->fMbxIs24Bit;
    s->current_task->cbSGEntry = s->fMbxIs24Bit ? sizeof(SGE24) : sizeof(SGE32);


    if (!s->fStrictRoundRobinMode)
    {
        qemu_log_mask(LOG_GUEST_ERROR, "simple mode\n");
        /* Search for a filled mailbox - stop if we have scanned all mailboxes. */
        uint8_t uMailboxPosCur = s->uMailboxOutgoingPositionCurrent;

        do
        {
            /* Fetch mailbox from guest memory. */
            GCPhysAddrMailboxCurrent = buslogicReadOutgoingMailbox(s, s->current_task);

             // Check the next mailbox.
            buslogicOutgoingMailboxAdvance(s);
        } while (   s->current_task->MailboxGuest.u.out.uActionCode == BUSLOGIC_MAILBOX_OUTGOING_ACTION_FREE
                 && uMailboxPosCur != s->uMailboxOutgoingPositionCurrent);
    }
    else
    {
        qemu_log_mask(LOG_GUEST_ERROR, "round robin mode\n");
        /* Fetch mailbox from guest memory. */
        GCPhysAddrMailboxCurrent = buslogicReadOutgoingMailbox(s,s->current_task);

    }


    //  * Check if the mailbox is actually loaded.
    //  * It might be possible that the guest notified us without
    //  * a loaded mailbox. Do nothing in that case but leave a
    //  * log entry.
    // qemu_log_mask(LOG_GUEST_ERROR, "action code = %i \n",TaskState->MailboxGuest.u.out.uActionCode);
    if (s->current_task->MailboxGuest.u.out.uActionCode == BUSLOGIC_MAILBOX_OUTGOING_ACTION_FREE)
    {

        if (s->current_task)
        {
            if (s->current_task->dataBuffer)
            {
                g_free(s->current_task->dataBuffer);
                s->current_task->dataBuffer = NULL;
                s->current_task->sglEntry.len = 0;
                s->current_task->sglEntry.base = 0;
                if (s->current_task->sglist)
                {
                    qemu_sglist_destroy(s->current_task->sglist);
                    g_free(s->current_task->sglist);
                    s->current_task->sglist = NULL;
                }
            }
            g_free(s->current_task);
            s->current_task = NULL;
        }
        // RTMemCacheFree(s->hTaskCache, pTaskState);
        // return VERR_NO_DATA;
        qemu_log_mask(LOG_GUEST_ERROR, "GO OUT\n");
        return -1;
    }

    /* We got the mailbox, mark it as free in the guest. */
    uint8_t uActionCode = BUSLOGIC_MAILBOX_OUTGOING_ACTION_FREE;
    unsigned uCodeOffs = s->current_task->fIs24Bit ? RT_OFFSETOF(Mailbox24, uCmdState) : RT_OFFSETOF(Mailbox32, u.out.uActionCode);
    // PDMDevHlpPCIPhysWrite(s->CTX_SUFF(pDevIns), GCPhysAddrMailboxCurrent + uCodeOffs, &uActionCode, sizeof(uActionCode));
    qemu_log_mask(LOG_GUEST_ERROR, "GCPhysAddrMailboxCurrent = 0x%p, ucodeoffs=%i uActionCode=%i\n",(void*)GCPhysAddrMailboxCurrent, uCodeOffs,uActionCode);
    // Mailbox32 *mbx = (Mailbox32* ) GCPhysAddrMailboxCurrent;
    cpu_physical_memory_write(GCPhysAddrMailboxCurrent + uCodeOffs, &uActionCode, sizeof(uActionCode));
    // qemu_log_mask(LOG_GUEST_ERROR, "pci_dma_write done\n");
    if (s->current_task->MailboxGuest.u.out.uActionCode == BUSLOGIC_MAILBOX_OUTGOING_ACTION_START_COMMAND)
    {
        result = buslogicDeviceSCSIRequestSetup(s, s->current_task);
    }
    else if (s->current_task->MailboxGuest.u.out.uActionCode == BUSLOGIC_MAILBOX_OUTGOING_ACTION_ABORT_COMMAND)
    {
        result = buslogicDeviceSCSIRequestAbort(s, s->current_task);
    }
    else
    {
        qemu_log_mask(LOG_GUEST_ERROR, "Invalid outgoing mailbox action code %u\n", s->current_task->MailboxGuest.u.out.uActionCode);
    }

    /* Advance to the next mailbox. */
    if (s->fStrictRoundRobinMode)
    {
        buslogicOutgoingMailboxAdvance(s);
    }


        qemu_log_mask(LOG_GUEST_ERROR, "DESTroy all resources ref to current task\n");
    if (s->current_task)
    {
        if (s->current_task->dataBuffer)
        {
            g_free(s->current_task->dataBuffer);
            s->current_task->dataBuffer = NULL;
            s->current_task->sglEntry.len = 0;
            s->current_task->sglEntry.base = 0;
            if (s->current_task->sglist)
            {
                qemu_sglist_destroy(s->current_task->sglist);
                g_free(s->current_task->sglist);
                s->current_task->sglist = NULL;
            }
        }
        g_free(s->current_task);
        s->current_task = NULL;
    }

    return result;
}


void buslogic_check_mailboxes(void *opaque)
{
    BuslogicState *s = (BuslogicState*)opaque;
    if (s->fNotificationSend == true && s->cMailboxesReady > 0)
    {
        s->fNotificationSend = false;
        s->cMailboxesReady = 0;
        qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);

        int rc;
        do
        {
            rc = buslogicProcessMailboxNext(s);
        } while (rc == 1);
    }

// anyway we need to alarm timer again
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    timer_mod_ns(s->mailboxCheckTimer, now + 10000);
}


static void buslogic_init_localram(BuslogicState *s)
{
    /*
     * These values are mostly from what I think is right
     * looking at the dmesg output from a Linux guest inside
     * a VMware server VM.
     *
     * So they don't have to be right :)
     */
    memset(s->localRam.u8View, 0, sizeof(HostAdapterLocalRam));
    s->localRam.structured.autoSCSIData.fLevelSensitiveInterrupt = true;
    s->localRam.structured.autoSCSIData.fParityCheckingEnabled = true;
    s->localRam.structured.autoSCSIData.fExtendedTranslation = true; /* Same as in geometry register. */
    s->localRam.structured.autoSCSIData.u16DeviceEnabledMask = ~0; /* All enabled. Maybe mask out non present devices? */
    s->localRam.structured.autoSCSIData.u16WidePermittedMask = ~0;
    s->localRam.structured.autoSCSIData.u16FastPermittedMask = ~0;
    s->localRam.structured.autoSCSIData.u16SynchronousPermittedMask = ~0;
    s->localRam.structured.autoSCSIData.u16DisconnectPermittedMask = ~0;
    s->localRam.structured.autoSCSIData.fStrictRoundRobinMode = s->fStrictRoundRobinMode;
    s->localRam.structured.autoSCSIData.u16UltraPermittedMask = ~0;
}

void buslogic_hard_reset(BuslogicState *s)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);
    qemu_log_mask(LOG_GUEST_ERROR, "reset status_register\n");
    // usleep(10000);
    s->status_register = 0;
    s->status_register  = BL_STAT_INREQ | BL_STAT_DACT;
    s->geo_register = BL_GEOM_XLATEN;
    s->is_status_read = false;
    s->current_command_byte = 0;
    s->current_reply_byte = 0;
    s->current_command = BL_NOOP;
    s->fStrictRoundRobinMode = false;
    s->current_command = BL_NOOP;
    s->uMailboxOutgoingPositionCurrent = 0;
    s->uMailboxIncomingPositionCurrent = 0;
    s->fIRQEnabled = true;
    buslogic_init_localram(s);
    s->fUseLocalRam = false;

    s->uPendingIntr = 0;
    buslogic_clear_interrupt(s);

    // int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    // s->mailboxCheckTimer = timer_new_ns(QEMU_CLOCK_REALTIME, buslogic_check_mailboxes, s);
    // timer_mod_ns(s->mailboxCheckTimer, now + 10000);
}

void buslogic_soft_reset(BuslogicState *s)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);
    s->status_register = 0;
    s->status_register |= BL_STAT_HARDY;
    s->data_in_register = 0;
    s->interrupt_register = 0;
    s->uPendingIntr = 0;
    s->geo_register = 0;
}

void buslogic_clear_interrupt(BuslogicState *s)
{
    PCIDevice *d = PCI_DEVICE(s);
    s->interrupt_register = 0;
    qemu_log_mask(LOG_GUEST_ERROR, "clear and call interrupt\n");
    pci_set_irq(d, 0);
    // PDMDevHlpPCISetIrq(pBusLogic->CTX_SUFF(pDevIns), 0, 0);
    /* If there's another pending interrupt, report it now. */
    if (s->uPendingIntr)
    {
        buslogic_set_interrupt(s, false, s->uPendingIntr);
        s->uPendingIntr = 0;
    }
}

void buslogic_int_reset(BuslogicState *s)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);
    buslogic_clear_interrupt(s);
}

// static void buslogic_sbus_reset(BuslogicState *s)
// {

// }

static void buslogic_scsi_reset(DeviceState *dev)
{
    BuslogicState *s = BUSLOGIC_BT958(dev);
    s->status_register |= BL_STAT_DFAIL;
    // buslogic_hard_reset(s);
    qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);
}


uint8_t buslogic_process_command(BuslogicState *s)
{
    uint8_t result = 1;
    bool fSuppressIrq = false;
    switch (s->current_command)
    {
        case BL_TEST_CMDC_INTERRUPT:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            break;
        }
        case BL_INITIALIZE_MAILBOX:
        {
            // qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            RequestInitMbx *req = (RequestInitMbx*) s->command_buffer;
            s->fMbxIs24Bit = true;
            s->mailbox = req->cMailbox;
            s->GCPhysAddrMailboxOutgoingBase = ADDR_TO_U32(req->aMailboxBaseAddr);

            /* The area for incoming mailboxes is right after the last entry of outgoing mailboxes. */
            s->GCPhysAddrMailboxIncomingBase = s->GCPhysAddrMailboxOutgoingBase + (s->mailbox * sizeof(Mailbox24));



            s->status_register &= ~BL_STAT_INREQ;
            s->reply_bytes_left = 0;
            break;
        }
        case BL_EXECUTE_MAILBOX_COMMAND:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            break;
        }
        case BL_EXECUTE_BIOS_COMMAND:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            break;
        }
        case BL_INQUIRE_BOARD_ID:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            /* The special option byte is important: If it is '0' or 'B', Windows NT drivers
             * for Adaptec AHA-154x may claim the adapter. The BusLogic drivers will claim
             * the adapter only when the byte is *not* '0' or 'B'.
             */
            s->reply_buffer[0] = 'A'; /* Firmware option bytes */
            s->reply_buffer[1] = 'A'; /* Special option byte */

            /* We report version 5.07B. This reply will provide the first two digits. */
            s->reply_buffer[2] = '5'; /* Major version 5 */
            s->reply_buffer[3] = '0'; /* Minor version 0 */
            s->reply_bytes_left = 4; /* Reply is 4 bytes long */
            break;
        }
        case BL_ENABLE_OUTGOING_MAILBOX_AVAILABLE_INTERRUPT:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            break;
        }
        case BL_SET_SCSI_SELECTION_TIMEOUT:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            break;
        }
        case BL_SET_PREEMPT_TIME_ON_BUS:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            break;
        }
        case BL_SET_TIME_OFF_BUS:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            break;
        }
        case BL_SET_BUS_TRANSFER_RATE:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            break;
        }
        case BL_INQUIRE_INSTALLED_DEVICES_ID_0_TO_7:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            break;
        }
        case BL_INQUIRE_CONFIGURATION:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            uint8_t pci_irq = PCI_DEVICE(s)->config[PCI_INTERRUPT_LINE];
            qemu_log_mask(LOG_GUEST_ERROR, "pci_irq = %i\n", pci_irq);
            ReplyInquireConfiguration *reply = (ReplyInquireConfiguration *)s->reply_buffer;
            memset(reply, 0, sizeof(ReplyInquireConfiguration));
            reply->uHostAdapterId = 7; /* The controller has always 7 as ID. */
            reply->fDmaChannel6  = 1;  /* DMA channel 6 is a good default. */
            /* The PCI IRQ is not necessarily representable in this structure.
             * If that is the case, the guest likely won't function correctly,
             * therefore we log a warning.
             */
            switch (pci_irq)
            {
                case 9:     reply->fIrqChannel9  = 1; break;
                case 10:    reply->fIrqChannel10 = 1; break;
                case 11:    reply->fIrqChannel11 = 1; break;
                case 12:    reply->fIrqChannel12 = 1; break;
                case 14:    reply->fIrqChannel14 = 1; break;
                case 15:    reply->fIrqChannel15 = 1; break;
                default:
                    qemu_log_mask(LOG_GUEST_ERROR, "Warning: PCI IRQ cannot be represented as ISA!\n");
                    break;
            }
            s->current_reply_byte = 0;
            s->reply_bytes_left = sizeof(ReplyInquireConfiguration);
            break;
        }
        case BL_ENABLE_TARGET_MODE:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            break;
        }
        case BL_INQUIRE_SETUP_INFORMATION:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            s->reply_bytes_left = s->command_buffer[0];
            ReplyInquireSetupInformation *reply = (ReplyInquireSetupInformation *) s->reply_buffer;
            memset(reply, 0, sizeof(ReplyInquireSetupInformation));
            reply->fSynchronousInitiationEnabled = true;
            reply->fParityCheckingEnabled = true;
            reply->cMailbox = s->mailbox;
            U32_TO_ADDR(reply->MailboxAddress, s->GCPhysAddrMailboxOutgoingBase);
            reply->uSignature = 'B';
            /* The 'D' signature prevents Adaptec's OS/2 drivers from getting too
             * friendly with BusLogic hardware and upsetting the HBA state.
             */
            reply->uCharacterD = 'D';      /* BusLogic model. */
            reply->uHostBusType = 'F';     /* PCI bus. */
            break;
        }
        case BL_WRITE_ADAPTER_LOCAL_RAM:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_READ_ADAPTER_LOCAL_RAM:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_WRITE_BUSMASTER_CHIP_FIFO:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_READ_BUSMASTER_CHIP_FIFO:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_ECHO_COMMAND_DATA:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_HOST_ADAPTER_DIAGNOSTIC:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_SET_ADAPTER_OPTIONS:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_INQUIRE_INSTALLED_DEVICES_ID_8_TO_15:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_INQUIRE_TARGET_DEVICES:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            /* Each bit which is set in the 16bit wide variable means a present device. */
            uint16_t u16TargetsPresentMask = 0;

            for (uint8_t i = 0; i < BUSLOGIC_MAX_DEVICES; i++)
            {
                if (s->deviceStates[i].fPresent)
                    u16TargetsPresentMask |= (1 << i);
            }
            s->reply_buffer[0] = (uint8_t)u16TargetsPresentMask;
            s->reply_buffer[1] = (uint8_t)(u16TargetsPresentMask >> 8);
            s->reply_bytes_left = 2;
            break;
        }
        case BL_DISABLE_HOST_ADAPTER_INTERRUPT:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            if (s->command_buffer[0] == 0)
                s->fIRQEnabled = false;
            else
                s->fIRQEnabled = true;
            /* No interrupt signaled regardless of enable/disable. */
            fSuppressIrq = true;
            break;
        }
        case BL_EXT_BIOS_INFO:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_UNLOCK_MAILBOX:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_INITIALIZE_EXTENDED_MAILBOX:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            RequestInitializeExtendedMailbox *request = (RequestInitializeExtendedMailbox *)s->command_buffer;

            s->fMbxIs24Bit = false;
            s->mailbox = request->cMailbox;
            s->GCPhysAddrMailboxOutgoingBase = (uint64_t)request->uMailboxBaseAddress;
            qemu_log_mask(LOG_GUEST_ERROR, "outgoing mailbox base = 0x%"PRIx64"\n",s->GCPhysAddrMailboxOutgoingBase);
            /* The area for incoming mailboxes is right after the last entry of outgoing mailboxes. */
            s->GCPhysAddrMailboxIncomingBase = (uint64_t)request->uMailboxBaseAddress + (s->mailbox * sizeof(Mailbox32));
            qemu_log_mask(LOG_GUEST_ERROR, "incoming mailbox base = 0x%"PRIx64"\n",s->GCPhysAddrMailboxIncomingBase);

            s->status_register &= ~BL_STAT_INREQ;
            s->reply_bytes_left = 0;
            break;
        }
        case BL_EXECUTE_SCSI_COMMAND:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_INQUIRE_FIRMWARE_VERSION_3RD_LETTER:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            s->reply_buffer[0] = '7';
            s->reply_bytes_left = 1;
            break;
        }
        case BL_INQUIRE_FIRMWARE_VERSION_LETTER:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            s->reply_buffer[0] = 'B';
            s->reply_bytes_left = 1;
            break;
        }
        case BL_INQUIRE_PCI_HOST_ADAPTER_INFORMATION:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            ReplyInquirePCIHostAdapterInformation *reply = (ReplyInquirePCIHostAdapterInformation *)s->reply_buffer;
            memset(reply, 0, sizeof(ReplyInquirePCIHostAdapterInformation));

            /* It seems VMware does not provide valid information here too, lets do the same :) */
            reply->InformationIsValid = 0;
            reply->IsaIOPort = s->uISABaseCode;
            reply->IRQ = PCI_DEVICE(s)->config[PCI_INTERRUPT_LINE];
            s->reply_bytes_left = sizeof(ReplyInquirePCIHostAdapterInformation);
            break;
        }
        case BL_INQUIRE_HOST_ADAPTER_MODEL_NUMBER:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);

            s->reply_bytes_left = s->command_buffer[0];
            memset(s->reply_buffer, ' ', s->reply_bytes_left);
            const char modelName[] = "958";
            int charsCnt =   (s->reply_bytes_left <= (sizeof(modelName) - 1))
                                   ? s->reply_bytes_left
                                   : sizeof(modelName) - 1;

            for (int i = 0; i < charsCnt; i++)
            {
                s->reply_buffer[i] = modelName[i];
            }
            break;
        }
        case BL_INQUIRE_SYNCHRONOUS_PERIOD:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            s->reply_bytes_left = s->command_buffer[0];

            for (uint8_t i = 0; i < s->reply_bytes_left; i++)
                s->reply_buffer[i] = 0; /** @todo Figure if we need something other here. It's not needed for the linux driver */
            break;
        }
        case BL_INQUIRE_EXTENDED_SETUP_INFORMATION:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x data = %02x\n",s->current_command, s->command_buffer[0]);
            s->reply_bytes_left = s->command_buffer[0];
            ReplyInquireExtendedSetupInformation *reply_info = (ReplyInquireExtendedSetupInformation *) s->reply_buffer;
            reply_info->uBusType = 'E';         /* EISA style */
            reply_info->u16ScatterGatherLimit = 8192;
            reply_info->cMailbox = s->mailbox;
            reply_info->uMailboxAddressBase = (uint32_t)s->GCPhysAddrMailboxOutgoingBase;
            reply_info->fLevelSensitiveInterrupt = true;
            reply_info->fHostWideSCSI = true;
            reply_info->fHostUltraSCSI = true;
            memcpy(reply_info->aFirmwareRevision, "07B", sizeof(reply_info->aFirmwareRevision));
            break;
        }
        case BL_ENABLE_STRICT_ROUND_ROBIN_MODE:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            if (s->command_buffer[0] == 0)
            {
                qemu_log_mask(LOG_GUEST_ERROR, "round robin mode disable\n");
                s->fStrictRoundRobinMode = false;
            }
            else if (s->command_buffer[0] == 1)
            {
                qemu_log_mask(LOG_GUEST_ERROR, "round robin mode enable\n");
                s->fStrictRoundRobinMode = true;
            }
            else
            {
                qemu_log_mask(LOG_GUEST_ERROR, "Invalid round robin mode %d\n", s->command_buffer[0]);
            }

            s->reply_bytes_left = 0;
            break;
        }
        case BL_STORE_HOST_ADAPTER_LOCAL_RAM:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_FETCH_HOST_ADAPTER_LOCAL_RAM:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            /*
             * First element in the command buffer contains start offset to read from
             * and second one the number of bytes to read.
             */
            // uint8_t uOffset = s->command_buffer[0];

            s->fUseLocalRam = true;

            s->current_reply_byte = s->command_buffer[0];
            s->reply_bytes_left = s->command_buffer[1];
            break;
        }
        case BL_STORE_LOCAL_DATA_IN_EEPROM:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_UPLOAD_AUTO_SCSI_CODE:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_MODIFY_IO_ADDRESS:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_SET_CCB_FORMAT:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            if (s->command_buffer[0] == 0)
            {
                qemu_log_mask(LOG_GUEST_ERROR, "extended LUN format disable\n");
                s->fExtendedLunCCBFormat = false;
            }
            else if (s->command_buffer[0] == 1)
            {
                qemu_log_mask(LOG_GUEST_ERROR, "extended LUN format enable\n");
                s->fExtendedLunCCBFormat = true;
            }
            else
                qemu_log_mask(LOG_GUEST_ERROR, "Invalid CCB format %d\n", s->command_buffer[0]);

            s->reply_bytes_left = 0;
            break;
        }
        case BL_WRITE_INQUIRY_BUFFER:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_READ_INQUIRY_BUFFER:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_FLASH_ROM_UPLOAD_DOWNLOAD:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_READ_SCAM_DATA:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        case BL_WRITE_SCAM_DATA:
        {
            qemu_log_mask(LOG_GUEST_ERROR, "need to run proc for command = 0x%02x\n",s->current_command);
            qemu_log_mask(LOG_GUEST_ERROR, "without implementation\n");
            break;
        }
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "!!!!!!!!!!!\n");
    };

    /* Set the data in ready bit in the status register in case the command has a reply. */
    if (s->reply_bytes_left)
        s->status_register |= BL_STAT_DIRRDY;
    else if (!s->command_bytes_left)
        buslogic_command_complete(s, fSuppressIrq);
    return result;
}



static void
buslogic_write_config(PCIDevice *pci, uint32_t addr, uint32_t val, int len)
{
    pci_default_write_config(pci, addr, val, len);
    msi_write_config(pci, addr, val, len);
}

static void buslogic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);

    k->realize = buslogic_scsi_realize;
    k->vendor_id = PCI_VENDOR_ID_BUSLOGIC;
    k->device_id = PCI_DEVICE_ID_BUSLOGIC_BT958;
    k->class_id = PCI_CLASS_STORAGE_RAID;
    k->revision = 0x00;
    k->subsystem_id = 0x1040;
    k->config_write = buslogic_write_config;
    dc->reset = buslogic_scsi_reset;
    //dc->vmsd = &vmstate_buslogic_scsi;

    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

static const TypeInfo buslogic_info = {
    .name          = TYPE_BUSLOGIC,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(BuslogicState),
    .class_init    = buslogic_class_init,
};


static void buslogic_register_types(void)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s\n",__FUNCTION__);
    type_register_static(&buslogic_info);

    // type_register_static(&lsi53c810_info);
}

type_init(buslogic_register_types)
