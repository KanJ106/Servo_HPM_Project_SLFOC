#ifndef S_RZT1_DEFINE_H
#define S_RZT1_DEFINE_H
/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/

/******************************************SCI********************************************/
/*
Serial mode register (SMR)
*/
/* Clock select (CKS[1:0]) */
#define _SCIF_CLOCK_SERICLK                     (0x0000U) /* SERICLK */
#define _SCIF_CLOCK_SERICLK_4                   (0x0001U) /* SERICLK/4 */
#define _SCIF_CLOCK_SERICLK_16                  (0x0002U) /* SERICLK/16 */
#define _SCIF_CLOCK_SERICLK_64                  (0x0003U) /* SERICLK/64 */
/* Stop bit length (STOP) */
#define _SCIF_STOP_1                            (0x0000U) /* 1 stop bit */
#define _SCIF_STOP_2                            (0x0008U) /* 2 stop bits */
/* Parity mode (PM) */
#define _SCIF_PARITY_EVEN                       (0x0000U) /* Parity even */
#define _SCIF_PARITY_ODD                        (0x0010U) /* Parity odd */
/* Parity enable (PE) */
#define _SCIF_PARITY_DISABLE                    (0x0000U) /* Parity disable */
#define _SCIF_PARITY_ENABLE                     (0x0020U) /* Parity enable */
/* Character length (CHR) */
#define _SCIF_DATA_LENGTH_8                     (0x0000U) /* Data length 8 bits */
#define _SCIF_DATA_LENGTH_7                     (0x0040U) /* Data length 7 bits */
/* Communications mode (CM) */
#define _SCIF_ASYNCHRONOUS_MODE                 (0x0000U) /* Asynchronous mode */
#define _SCIF_CLOCK_SYNCHRONOUS_MODE            (0x0080U) /* Clock synchronous mode */

/*
    Serial control register (SCR)
*/
/* Clock enable (CKE[1:0]) */
#define _SCIF_INTERNAL_SCK_UNUSED               (0x0000U) /* Internal clock selected, SCK pin unused */
#define _SCIF_INTERNAL_SCK_OUTPUT               (0x0001U) /* Internal clock selected, SCK pin as clock output */
/* Clock enable (CKE[1:0]) for clock synchronous mode */
#define _SCIF_INTERNAL_SCK_OUTPUT_SYNC          (0x0000U) /* Internal clock, SCK pin is used for clock output */
#define _SCIF_EXTERNAL_SCK_INPUT_SYNC           (0x0002U) /* External clock, SCK pin is used for clock input */
/* Transmit end interrupt enable (TEIE) */
#define _SCIF_TEI_INTERRUPT_DISABLE             (0x0000U) /* TEI interrupt request disable */
#define _SCIF_TEI_INTERRUPT_ENABLE              (0x0004U) /* TEI interrupt request enable */
/* Receive error interrupt enable (REIE) */
#define _SCIF_ERI_BRI_INTERRUPT_DISABLE         (0x0000U) /* Disable receive-error interrupt and break interrupt */
#define _SCIF_ERI_BRI_INTERRUPT_ENABLE          (0x0008U) /* Enable receive-error interrupt and break interrupt */
/* Receive enable (RE) */
#define _SCIF_RECEIVE_DISABLE                   (0x0000U) /* Disable receive mode */
#define _SCIF_RECEIVE_ENABLE                    (0x0010U) /* Enable receive mode */
/* Transmit enable (TE) */
#define _SCIF_TRANSMIT_DISABLE                  (0x0000U) /* Disable transmit mode */
#define _SCIF_TRANSMIT_ENABLE                   (0x0020U) /* Enable transmit mode */
/* Receive interrupt enable (RIE) */
#define _SCIF_RXI_ERI_DISABLE                   (0x0000U) /* Disable RXI and ERI interrupt requests */
#define _SCIF_RXI_ERI_ENABLE                    (0x0040U) /* Enable RXI and ERI interrupt requests */
/* Transmit interrupt enable (TIE) */
#define _SCIF_TXI_DISABLE                       (0x0000U) /* Disable TXI interrupt requests */
#define _SCIF_TXI_ENABLE                        (0x0080U) /* Enable TXI interrupt requests */

/*
    FIFO control register (FCR)
*/
/* Loop-Back test (LOOP) */
#define _SCIF_LOOPBACK_DISABLE                  (0x0000U) /* Loop back test is disabled */
#define _SCIF_LOOPBACK_ENABLE                   (0x0001U) /* Loop back test is enabled */
/* Receive FIFO Data Register Reset (RFRST) */
#define _SCIF_RX_FIFO_RESET_DISABLE             (0x0000U) /* FRDR reset operation is disabled */
#define _SCIF_RX_FIFO_RESET_ENABLE              (0x0002U) /* FRDR reset operation is enabled */
/* Transmit FIFO Data Register Reset (TFRST) */
#define _SCIF_TX_FIFO_RESET_DISABLE             (0x0000U) /* FTDR reset operation is disabled */
#define _SCIF_TX_FIFO_RESET_ENABLE              (0x0004U) /* FTDR reset operation is enabled */
/* Modem control enable (MCE) */
#define _SCIF_MODEM_CONTROL_DISABLE             (0x0000U) /* Model signal is disabled */
#define _SCIF_MODEM_CONTROL_ENABLE              (0x0008U) /* Model signal is enabled */
/* Transmit FIFO Data Trigger Number (TTRG[1:0]) */
#define _SCIF_TX_TRIGGER_NUMBER_8               (0x0000U) /* 8 (or 8 when TDFE flag is 1) */
#define _SCIF_TX_TRIGGER_NUMBER_4               (0x0010U) /* 4 (or 12 when TDFE flag is 1) */
#define _SCIF_TX_TRIGGER_NUMBER_2               (0x0020U) /* 2 (or 14 when TDFE flag is 1) */
#define _SCIF_TX_TRIGGER_NUMBER_0               (0x0030U) /* 0 (or 16 when TDFE flag is 1) */
/* Receive FIFO Data Trigger Number (RTRG[1:0]) */
#define _SCIF_RX_TRIGGER_NUMBER_1               (0x0000U) /* 1 */
#define _SCIF_RX_TRIGGER_NUMBER_4               (0x0040U) /* 4 (for asynchronous mode) */
#define _SCIF_RX_TRIGGER_NUMBER_2               (0x0040U) /* 2 (for clock synchronous mode */
#define _SCIF_RX_TRIGGER_NUMBER_8               (0x0080U) /* 8 */
#define _SCIF_RX_TRIGGER_NUMBER_14              (0x00C0U) /* 14 */
/* RTS# Output Active Trigger Number Select (RSTRG[2:0]) */
#define _SCIF_RTS_TRIGGER_NUMBER_15             (0x0000U) /* 15 */
#define _SCIF_RTS_TRIGGER_NUMBER_1              (0x0100U) /* 1 */
#define _SCIF_RTS_TRIGGER_NUMBER_4              (0x0200U) /* 4 */
#define _SCIF_RTS_TRIGGER_NUMBER_6              (0x0300U) /* 6 */
#define _SCIF_RTS_TRIGGER_NUMBER_8              (0x0400U) /* 8 */
#define _SCIF_RTS_TRIGGER_NUMBER_10             (0x0500U) /* 10 */
#define _SCIF_RTS_TRIGGER_NUMBER_12             (0x0600U) /* 12 */
#define _SCIF_RTS_TRIGGER_NUMBER_14             (0x0700U) /* 14 */

/*
    Serial port register (SPTR)
*/
/* Serial Port Break Data (SPB2DT) */
#define _SCIF_SERIAL_BREAK_DATA_LOW             (0x0000U) /* Input/output data is at low */
#define _SCIF_SERIAL_BREAK_DATA_HIGH            (0x0001U) /* Input/output data is at high */
/* Serial Port Break input/output (SPB2IO) */
#define _SCIF_SERIAL_BREAK_TXD_NO_OUTPUT        (0x0000U) /* SPB2DT bit value is not output to TXD pin */
#define _SCIF_SERIAL_BREAK_TXD_OUTPUT           (0x0002U) /* SPB2DT bit value is output to TXD pin */
/* SCK Port Data (SCKDT) */
#define _SCIF_SCK_DATA_LOW                      (0x0000U) /* Input/output data is at low */
#define _SCIF_SCK_DATA_HIGH                     (0x0004U) /* Input/output data is at high */
/* SCK Port input/output (SCKIO) */
#define _SCIF_SCK_PORT_NO_OUTPUT                (0x0000U) /* SCKDT bit value is not output to SCK pin */
#define _SCIF_SCK_PORT_OUTPUT                   (0x0008U) /* SCKDT bit value is output to SCK pin */
/* CTS# Port Data Select (CTS2DT) */
#define _SCIF_CTS_DATA_0                        (0x0000U) /* Set b4 to 0. Controls CTS# pin with MCE, CTS2IO bit */
#define _SCIF_CTS_DATA_1                        (0x0010U) /* Set b4 to 1. Controls CTS# pin with MCE, CTS2IO bit */
/* CTS# Port Output Specify (CTS2IO) */
#define _SCIF_CTS_OUTPUT_0                      (0x0000U) /* Set b5 to 0. Controls CTS# pin with MCE, CTS2IO bit */
#define _SCIF_CTS_OUTPUT_1                      (0x0020U) /* Set b5 to 1. Controls CTS# pin with MCE, CTS2IO bit */
/* RTS# Port Data Select (RTS2DT) */
#define _SCIF_RTS_DATA_0                        (0x0000U) /* Set b6 to 0. Controls RTS# pin with MCE, RTS2IO bit */
#define _SCIF_RTS_DATA_1                        (0x0040U) /* Set b6 to 1. Controls RTS# pin with MCE, RTS2IO bit */
/* RTS# Port Output Specify (RTS2IO) */
#define _SCIF_RTS_OUTPUT_0                      (0x0000U) /* Set b7 to 0. Controls RTS# pin with MCE, RTS2IO bit */
#define _SCIF_RTS_OUTPUT_1                      (0x0080U) /* Set b7 to 1. Controls RTS# pin with MCE, RTS2IO bit */

/*
    FIFO Trigger Control Register (FTCR)
*/
/* Transmit FIFO Data Trigger Number (TFTC[4:0]) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_0             (0x0000U) /* 0 (no transmit data trigger) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_1             (0x0001U) /* 1 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_2             (0x0002U) /* 2 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_3             (0x0003U) /* 3 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_4             (0x0004U) /* 4 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_5             (0x0005U) /* 5 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_6             (0x0006U) /* 6 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_7             (0x0007U) /* 7 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_8             (0x0008U) /* 8 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_9             (0x0009U) /* 9 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_10            (0x000AU) /* 10 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_11            (0x000BU) /* 11 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_12            (0x000CU) /* 12 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_13            (0x000DU) /* 13 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_14            (0x000EU) /* 14 (transmit data triggers) */
#define _SCIF_TX_FIFO_TRIGGER_NUM_15            (0x000FU) /* 15 (transmit data triggers) */
/* Transmit Trigger Select (TTRGS) */
#define _SCIF_TX_TRIGGER_TTRG_VALID             (0x0000U) /* TTRG[1:0] bits in FCR are valid */
#define _SCIF_TX_TRIGGER_TFTC_VALID             (0x0080U) /* TFTC[4:0] bits in FTCR are valid */
/* Receive FIFO Data Trigger Number (RFTC[4:0]) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_1             (0x0100U) /* 1 (receive data trigger) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_2             (0x0200U) /* 2 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_3             (0x0300U) /* 3 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_4             (0x0400U) /* 4 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_5             (0x0500U) /* 5 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_6             (0x0600U) /* 6 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_7             (0x0700U) /* 7 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_8             (0x0800U) /* 8 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_9             (0x0900U) /* 9 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_10            (0x0A00U) /* 10 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_11            (0x0B00U) /* 11 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_12            (0x0C00U) /* 12 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_13            (0x0D00U) /* 13 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_14            (0x0E00U) /* 14 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_15            (0x0F00U) /* 15 (receive data triggers) */
#define _SCIF_RX_FIFO_TRIGGER_NUM_16            (0x1000U) /* 16 (receive data triggers) */
/* Transmit Trigger Select (RTRGS) */
#define _SCIF_RX_TRIGGER_RTRG_VALID             (0x0000U) /* RTRG[1:0] bits in FCR are valid */
#define _SCIF_RX_TRIGGER_RFTC_VALID             (0x8000U) /* RFTC[4:0] bits in FTCR are valid */

/*
    Serial extended mode register (SEMR)
*/
/* Asynchronous base clock select (ABCS0) */
#define _SCIF_16_BASE_CLOCK                     (0x00U) /* Selects 16 base clock cycles for 1 bit period */
#define _SCIF_8_BASE_CLOCK                      (0x01U) /* Selects 8 base clock cycles for 1 bit period */
/* Noise Cancellation Enable (NFEN) */
#define _SCIF_NOISE_FILTER_DISABLE              (0x00U) /* Noise cancellation for the RxD pin input is disabled */
#define _SCIF_NOISE_FILTER_ENABLE               (0x04U) /* Noise cancellation for the RxD pin input is enabled */
/* Data Transfer Direction Select (DIR) */
#define _SCIF_DATA_TRANSFER_LSB_FIRST           (0x00U) /* Transmits the data in FTDR by the LSB-first method */
#define _SCIF_DATA_TRANSFER_MSB_FIRST           (0x08U) /* Transmits the data in FTDR by the MSB-first method */
/* Modulation Duty Register Select (MDDRS) */
#define _SCIF_BRR_USED                          (0x00U) /* BRR register can be accessed */
#define _SCIF_MDDR_USED                         (0x10U) /* MDDR register can be accessed. */
/* Bit Rate Modulation Enable (BRME) */
#define _SCIF_BIT_RATE_MODULATION_DISABLE       (0x00U) /* Bit rate modulation function is disabled */
#define _SCIF_BIT_RATE_MODULATION_ENABLE        (0x20U) /* Bit rate modulation function is enabled */
/* Baud Rate Generator Double-Speed Mode Select (BGDM) */
#define _SCIF_BAUDRATE_SINGLE                   (0x00U) /* Baud rate generator outputs normal frequency */
#define _SCIF_BAUDRATE_DOUBLE                   (0x80U) /* Baud rate generator doubles output frequency */

/*
    Interrupt Source Priority Register n (PRLn)
*/
/* Interrupt Priority Level Select (PRL[3:0]) */
#define _SCIF_PRIORITY_LEVEL0                   (0x00000000UL) /* Level 0 (highest) */
#define _SCIF_PRIORITY_LEVEL1                   (0x00000001UL) /* Level 1 */
#define _SCIF_PRIORITY_LEVEL2                   (0x00000002UL) /* Level 2 */
#define _SCIF_PRIORITY_LEVEL3                   (0x00000003UL) /* Level 3 */
#define _SCIF_PRIORITY_LEVEL4                   (0x00000004UL) /* Level 4 */
#define _SCIF_PRIORITY_LEVEL5                   (0x00000005UL) /* Level 5 */
#define _SCIF_PRIORITY_LEVEL6                   (0x00000006UL) /* Level 6 */
#define _SCIF_PRIORITY_LEVEL7                   (0x00000007UL) /* Level 7 */
#define _SCIF_PRIORITY_LEVEL8                   (0x00000008UL) /* Level 8 */
#define _SCIF_PRIORITY_LEVEL9                   (0x00000009UL) /* Level 9 */
#define _SCIF_PRIORITY_LEVEL10                  (0x0000000AUL) /* Level 10 */
#define _SCIF_PRIORITY_LEVEL11                  (0x0000000BUL) /* Level 11 */
#define _SCIF_PRIORITY_LEVEL12                  (0x0000000CUL) /* Level 12 */
#define _SCIF_PRIORITY_LEVEL13                  (0x0000000DUL) /* Level 13 */
#define _SCIF_PRIORITY_LEVEL14                  (0x0000000EUL) /* Level 14 */
#define _SCIF_PRIORITY_LEVEL15                  (0x0000000FUL) /* Level 15 */

/* FIFO buffer maximum size */
#define _SCIF_FIFO_MAX_SIZE                     (0x10U) /* Size of 16-stage FIFO buffer */

/******************************************SCI********************************************/

/******************************************SPI********************************************/
/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/*
    RSPI Control Register (SPCR)
*/
/* RSPI Mode Select (SPMS) */
#define _RSPI_MODE_SPI                       (0x00U) /* SPI operation (four-wire method) */
#define _RSPI_MODE_CLOCK_SYNCHRONOUS         (0x01U) /* Clock synchronous operation (three-wire method) */
/* Communications Operating Mode Select (TXMD) */
#define _RSPI_FULL_DUPLEX_SYNCHRONOUS        (0x00U) /* Full-duplex synchronous serial communications */
#define _RSPI_TRANSMIT_ONLY                  (0x02U) /* Serial communications with transmit only operations */
/* Mode Fault Error Detection Enable (MODFEN) */
#define _RSPI_MODE_FAULT_DETECT_DISABLED     (0x00U) /* Disables the detection of mode fault error */
#define _RSPI_MODE_FAULT_DETECT_ENABLED      (0x04U) /* Enables the detection of mode fault error */
/* RSPI Master/Slave Mode Select (MSTR) */
#define _RSPI_SLAVE_MODE                     (0x00U) /* Slave mode */
#define _RSPI_MASTER_MODE                    (0x08U) /* Master mode */
/* RSPI Error Interrupt Enable (SPEIE) */
#define _RSPI_ERROR_INTERRUPT_DISABLED       (0x00U) /* Disables the generation of RSPI error interrupt */
#define _RSPI_ERROR_INTERRUPT_ENABLED        (0x10U) /* Enables the generation of RSPI error interrupt */
/* RSPI Transmit Interrupt Enable (SPTIE) */
#define _RSPI_TRANSMIT_INTERRUPT_DISABLED    (0x00U) /* Disables the generation of RSPI transmit interrupt */
#define _RSPI_TRANSMIT_INTERRUPT_ENABLED     (0x20U) /* Enables the generation of RSPI transmit interrupt */
/* RSPI Function Enable (SPE) */
#define _RSPI_FUNCTION_DISABLED              (0x00U) /* Disables the RSPI function */
#define _RSPI_FUNCTION_ENABLED               (0x40U) /* Enables the RSPI function */
/* RSPI Receive Interrupt Enable (SPRIE) */
#define _RSPI_RECEIVE_INTERRUPT_DISABLED     (0x00U) /* Disables the generation of RSPI receive interrupt */
#define _RSPI_RECEIVE_INTERRUPT_ENABLED      (0x80U) /* Enables the generation of RSPI receive interrupt */

/*
    RSPI Slave Select Polarity Register (SSLP)
*/
/* SSL0 Signal Polarity Setting (SSL0P) */
#define _RSPI_SSL0_POLARITY_LOW              (0x00U) /* SSL0 signal is active low */
#define _RSPI_SSL0_POLARITY_HIGH             (0x01U) /* SSL0 signal is active high */
/* SSL1 Signal Polarity Setting (SSL1P) */
#define _RSPI_SSL1_POLARITY_LOW              (0x00U) /* SSL1 signal is active low */
#define _RSPI_SSL1_POLARITY_HIGH             (0x02U) /* SSL1 signal is active high */
/* SSL2 Signal Polarity Setting (SSL2P) */
#define _RSPI_SSL2_POLARITY_LOW              (0x00U) /* SSL2 signal is active low */
#define _RSPI_SSL2_POLARITY_HIGH             (0x04U) /* SSL2 signal is active high */
/* SSL3 Signal Polarity Setting (SSL3P) */
#define _RSPI_SSL3_POLARITY_LOW              (0x00U) /* SSL3 signal is active low */
#define _RSPI_SSL3_POLARITY_HIGH             (0x08U) /* SSL3 signal is active high */

/*
    RSPI Pin Control Register (SPPCR)
*/
/* RSPI Loopback (SPLP) */
#define _RSPI_LOOPBACK_DISABLED              (0x00U) /* Normal mode */
#define _RSPI_LOOPBACK_ENABLED               (0x01U) /* Loopback mode (reversed transmit data = receive data) */
/* RSPI Loopback 2 (SPLP2) */
#define _RSPI_LOOPBACK2_DISABLED             (0x00U) /* Normal mode */
#define _RSPI_LOOPBACK2_ENABLED              (0x02U) /* Loopback mode (transmit data = receive data) */
/* Output pin mode (SPOM) */
#define _RSPI_OUTPUT_PIN_CMOS                (0x00U) /* CMOS output */
#define _RSPI_OUTPUT_PIN_OPEN_DRAIN          (0x04U) /* Open-drain output */
/* MOSI Idle Fixed Value (MOIFV) */
#define _RSPI_MOSI_LEVEL_LOW                 (0x00U) /* Level output on MOSIA during idling corresponds to low */
#define _RSPI_MOSI_LEVEL_HIGH                (0x10U) /* Level output on MOSIA during idling corresponds to high */
/* MOSI Idle Value Fixing Enable (MOIFE) */
#define _RSPI_MOSI_FIXING_PREV_TRANSFER      (0x00U) /* MOSI output value equals final data from previous transfer */
#define _RSPI_MOSI_FIXING_MOIFV_BIT          (0x20U) /* MOSI output value equals the value set in the MOIFV bit */

/*
    RSPI Sequence Control Register (SPSCR)
*/
/* RSPI Sequence Length Specification (SPSLN[2:0]) */
#define _RSPI_SEQUENCE_LENGTH_1              (0x00U) /* 0 -> 0... */
#define _RSPI_SEQUENCE_LENGTH_2              (0x01U) /* 0 -> 1 -> 0... */
#define _RSPI_SEQUENCE_LENGTH_3              (0x02U) /* 0 -> 1 -> 2 -> 0... */
#define _RSPI_SEQUENCE_LENGTH_4              (0x03U) /* 0 -> 1 -> 2 -> 3 -> 0... */
#define _RSPI_SEQUENCE_LENGTH_5              (0x04U) /* 0 -> 1 -> 2 -> 3 -> 4 -> 0... */
#define _RSPI_SEQUENCE_LENGTH_6              (0x05U) /* 0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 0... */
#define _RSPI_SEQUENCE_LENGTH_7              (0x06U) /* 0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 0... */
#define _RSPI_SEQUENCE_LENGTH_8              (0x07U) /* 0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 0... */

/*
    RSPI Data Control Register (SPDCR)
*/
/* Number of Frames Specification (SPFC[1:0]) */
#define _RSPI_FRAMES_1                       (0x00U) /* 1 frame */
#define _RSPI_FRAMES_2                       (0x01U) /* 2 frames */
#define _RSPI_FRAMES_3                       (0x02U) /* 3 frames */
#define _RSPI_FRAMES_4                       (0x03U) /* 4 frames */
/* RSPI Receive/Transmit Data Selection (SPRDTD) */
#define _RSPI_READ_SPDR_RX_BUFFER            (0x00U) /* read SPDR values from receive buffer */
#define _RSPI_READ_SPDR_TX_BUFFER            (0x10U) /* read SPDR values from transmit buffer (transmit buffer empty) */
/* RSPI Longword Access/Word Access Specification (SPLW) */ 
#define _RSPI_ACCESS_WORD                    (0x00U) /* SPDR is accessed in words */
#define _RSPI_ACCESS_LONGWORD                (0x20U) /* SPDR is accessed in longwords */

/*
    RSPI Clock Delay Register (SPCKD)
*/
/* RSPCK Delay Setting (SCKDL[2:0]) */
#define _RSPI_RSPCK_DELAY_1                  (0x00U) /* 1 RSPCK */
#define _RSPI_RSPCK_DELAY_2                  (0x01U) /* 2 RSPCK */
#define _RSPI_RSPCK_DELAY_3                  (0x02U) /* 3 RSPCK */
#define _RSPI_RSPCK_DELAY_4                  (0x03U) /* 4 RSPCK */
#define _RSPI_RSPCK_DELAY_5                  (0x04U) /* 5 RSPCK */
#define _RSPI_RSPCK_DELAY_6                  (0x05U) /* 6 RSPCK */
#define _RSPI_RSPCK_DELAY_7                  (0x06U) /* 7 RSPCK */
#define _RSPI_RSPCK_DELAY_8                  (0x07U) /* 8 RSPCK */

/*
    RSPI Slave Select Negation Delay Register (SSLND)
*/
/* SSL Negation Delay Setting (SLNDL[2:0]) */
#define _RSPI_SSL_NEGATION_DELAY_1           (0x00U) /* 1 RSPCK */
#define _RSPI_SSL_NEGATION_DELAY_2           (0x01U) /* 2 RSPCK */
#define _RSPI_SSL_NEGATION_DELAY_3           (0x02U) /* 3 RSPCK */
#define _RSPI_SSL_NEGATION_DELAY_4           (0x03U) /* 4 RSPCK */
#define _RSPI_SSL_NEGATION_DELAY_5           (0x04U) /* 5 RSPCK */
#define _RSPI_SSL_NEGATION_DELAY_6           (0x05U) /* 6 RSPCK */
#define _RSPI_SSL_NEGATION_DELAY_7           (0x06U) /* 7 RSPCK */
#define _RSPI_SSL_NEGATION_DELAY_8           (0x07U) /* 8 RSPCK */

/*
    RSPI Next-Access Delay Register (SPND)
*/
/* RSPI Next-Access Delay Setting (SPNDL[2:0]) */
#define _RSPI_NEXT_ACCESS_DELAY_1            (0x00U) /* 1 RSPCK + 2 SERICLK */
#define _RSPI_NEXT_ACCESS_DELAY_2            (0x01U) /* 2 RSPCK + 2 SERICLK */
#define _RSPI_NEXT_ACCESS_DELAY_3            (0x02U) /* 3 RSPCK + 2 SERICLK */
#define _RSPI_NEXT_ACCESS_DELAY_4            (0x03U) /* 4 RSPCK + 2 SERICLK */
#define _RSPI_NEXT_ACCESS_DELAY_5            (0x04U) /* 5 RSPCK + 2 SERICLK */
#define _RSPI_NEXT_ACCESS_DELAY_6            (0x05U) /* 6 RSPCK + 2 SERICLK */
#define _RSPI_NEXT_ACCESS_DELAY_7            (0x06U) /* 7 RSPCK + 2 SERICLK */
#define _RSPI_NEXT_ACCESS_DELAY_8            (0x07U) /* 8 RSPCK + 2 SERICLK */

/*
    RSPI Control Register 2 (SPCR2)
*/
/* Parity Enable (SPPE) */
#define _RSPI_PARITY_DISABLE                 (0x00U) /* Does not add parity bit to transmit data */
#define _RSPI_PARITY_ENABLE                  (0x01U) /* Adds the parity bit to transmit data */
/* Parity Mode (SPOE) */
#define _RSPI_PARITY_EVEN                    (0x00U) /* Selects even parity for use in transmission and reception */
#define _RSPI_PARITY_ODD                     (0x02U) /* Selects odd parity for use in transmission and reception */
/* RSPI Idle Interrupt Enable (SPIIE) */
#define _RSPI_IDLE_INTERRUPT_DISABLED        (0x00U) /* Disables the generation of RSPI idle interrupt */
#define _RSPI_IDLE_INTERRUPT_ENABLED         (0x04U) /* Enables the generation of RSPI idle interrupt */
/* Parity Self-Testing (PTE) */
#define _RSPI_SELF_TEST_DISABLED             (0x00U) /* Disables the self-diagnosis function of the parity circuit */
#define _RSPI_SELF_TEST_ENABLED              (0x08U) /* Enables the self-diagnosis function of the parity circuit */
/* RSPCK Auto-Stop Function Enable (SCKASE) */
#define _RSPI_AUTO_STOP_DISABLED             (0x00U) /* Disables the RSPCK auto-stop function */
#define _RSPI_AUTO_STOP_ENABLED              (0x10U) /* Enables the RSPCK auto-stop function */

/*
    RSPI Command Registers 0 to 7 (SPCMD0 to SPCMD7)
*/
/* RSPCK Phase Setting (CPHA) */
#define _RSPI_RSPCK_SAMPLING_ODD           (0x0000U) /* Data sampling on odd edge, data variation on even edge */
#define _RSPI_RSPCK_SAMPLING_EVEN          (0x0001U) /* Data variation on odd edge, data sampling on even edge */
/* RSPCK Polarity Setting (CPOL) */
#define _RSPI_RSPCK_POLARITY_LOW           (0x0000U) /* RSPCK is low when idle */
#define _RSPI_RSPCK_POLARITY_HIGH          (0x0002U) /* RSPCK is high when idle */
/* Bit Rate Division Setting (BRDV[1:0]) */
#define _RSPI_BASE_BITRATE_1               (0x0000U) /* These bits select the base bit rate */
#define _RSPI_BASE_BITRATE_2               (0x0004U) /* These bits select the base bit rate divided by 2 */
#define _RSPI_BASE_BITRATE_4               (0x0008U) /* These bits select the base bit rate divided by 4 */
#define _RSPI_BASE_BITRATE_8               (0x000CU) /* These bits select the base bit rate divided by 8 */
/* SSL Signal Assertion Setting (SSLA[2:0]) */
#define _RSPI_SIGNAL_ASSERT_SSL0           (0x0000U) /* SSL0 */
#define _RSPI_SIGNAL_ASSERT_SSL1           (0x0010U) /* SSL1 */
#define _RSPI_SIGNAL_ASSERT_SSL2           (0x0020U) /* SSL2 */
#define _RSPI_SIGNAL_ASSERT_SSL3           (0x0030U) /* SSL3 */
/* SSL Signal Level Keeping (SSLKP) */
#define _RSPI_SSL_KEEP_DISABLE             (0x0000U) /* Negates all SSL signals upon completion of transfer */
#define _RSPI_SSL_KEEP_ENABLE              (0x0080U) /* Keep SSL level from end of transfer till next access */
/* RSPI Data Length Setting (SPB[3:0]) */
#define _RSPI_DATA_LENGTH_BITS_8           (0x0400U) /* 8 bits */
#define _RSPI_DATA_LENGTH_BITS_9           (0x0800U) /* 9 bits */
#define _RSPI_DATA_LENGTH_BITS_10          (0x0900U) /* 10 bits */
#define _RSPI_DATA_LENGTH_BITS_11          (0x0A00U) /* 11 bits */
#define _RSPI_DATA_LENGTH_BITS_12          (0x0B00U) /* 12 bits */
#define _RSPI_DATA_LENGTH_BITS_13          (0x0C00U) /* 13 bits */
#define _RSPI_DATA_LENGTH_BITS_14          (0x0D00U) /* 14 bits */
#define _RSPI_DATA_LENGTH_BITS_15          (0x0E00U) /* 15 bits */
#define _RSPI_DATA_LENGTH_BITS_16          (0x0F00U) /* 16 bits */
#define _RSPI_DATA_LENGTH_BITS_20          (0x0000U) /* 20 bits */
#define _RSPI_DATA_LENGTH_BITS_24          (0x0100U) /* 24 bits */
#define _RSPI_DATA_LENGTH_BITS_32          (0x0200U) /* 32 bits */
/* RSPI LSB First (LSBF) */
#define _RSPI_MSB_FIRST                    (0x0000U) /* MSB first */
#define _RSPI_LSB_FIRST                    (0x1000U) /* LSB first */
/* RSPI Next-Access Delay Enable (SPNDEN) */
#define _RSPI_NEXT_ACCESS_DELAY_DISABLE    (0x0000U) /* Next-access delay of 1 RSPCK + 2 SERICLK */
#define _RSPI_NEXT_ACCESS_DELAY_ENABLE     (0x2000U) /* Next-access delay equal to setting of SPND register */
/* SSL Negation Delay Setting Enable (SLNDEN) */
#define _RSPI_NEGATION_DELAY_DISABLE       (0x0000U) /* SSL negation delay of 1 RSPCK */
#define _RSPI_NEGATION_DELAY_ENABLE        (0x4000U) /* SSL negation delay equal to setting of SSLND register */
/* RSPCK Delay Setting Enable (SCKDEN) */
#define _RSPI_RSPCK_DELAY_DISABLE          (0x0000U) /* RSPCK delay of 1 RSPCK */
#define _RSPI_RSPCK_DELAY_ENABLE           (0x8000U) /* RSPCK delay equal to setting of the SPCKD register */

/******************************************SPI********************************************/

/******************************************DMA********************************************/
/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/*
    Channel Control Register n (CHCTRL_n)
*/
/* DMA Activation Enable (SETEN) */
#define _DMAC_CHCTRL_ENABLE_SET                        (0x00000001UL)
/* DMA Activation Enable Clear (CLREN) */
#define _DMAC_CHCTRL_ENABLE_CLEAR                      (0x00000002UL)
/* Software Reset (SWRST) */
#define _DMAC_CHCTRL_SW_RESET                          (0x00000008UL)
/* DMA Transfer Request Clear (CLRRQ) */
#define _DMAC_CHCTRL_RQST_CLEAR                        (0x00000010UL)
/* DMA Transfer END Clear (CLREND) */
#define _DMAC_CHCTRL_END_CLEAR                         (0x00000020UL)
/* DER Clear (CLRDER) */
#define _DMAC_CHCTRL_DER_CLEAR                         (0x00000080UL)
/* Suspend Request (SETSUS) */
#define _DMAC_CHCTRL_SUSPEND_SET                       (0x00000100UL)
/* Suspend Clear (CLRSUS) */
#define _DMAC_CHCTRL_SUSPEND_CLEAR                     (0x00000200UL)
/* REN Set Enable (SETREN) */
#define _DMAC_CHCTRL_REN_SET                           (0x00001000UL)
/* Software Forced Ejection Request (SETSSWPRQ) */
#define _DMAC_CHCTRL_SW_FORCE_EJECT                    (0x00004000UL)
/* Interrupt Request Mask (SETINTM) */
#define _DMAC_CHCTRL_MASK_COMPLETE_INT                 (0x00010000UL)
/* Interrupt Request Mask Clear (CLRINTM) */
#define _DMAC_CHCTRL_COMPLETE_INT_MASK_CLEAR           (0x00020000UL)
/* DMA Activation Request Mask (SETDMARQM) */
#define _DMAC_CHCTRL_REQUEST_MASK                      (0x00040000UL)
/* DMA Activation Request Mask Clear (CLRDMARQM) */
#define _DMAC_CHCTRL_REQUEST_MASK_CLEAR                (0x00080000UL)

/*
    Channel Configuration Register n (CHCFG_n)
*/
/* Pin Select (SEL[2:0]) */
#define _DMAC_CHCFG_SEL_CHANNEL_0                      (0x00000000UL)
#define _DMAC_CHCFG_SEL_CHANNEL_1                      (0x00000001UL)
#define _DMAC_CHCFG_SEL_CHANNEL_2                      (0x00000002UL)
#define _DMAC_CHCFG_SEL_CHANNEL_3                      (0x00000003UL)
#define _DMAC_CHCFG_SEL_CHANNEL_4                      (0x00000004UL)
#define _DMAC_CHCFG_SEL_CHANNEL_5                      (0x00000005UL)
#define _DMAC_CHCFG_SEL_CHANNEL_6                      (0x00000006UL)
#define _DMAC_CHCFG_SEL_CHANNEL_7                      (0x00000007UL)
#define _DMAC_CHCFG_SEL_CHANNEL_8                      (0x00000000UL)
#define _DMAC_CHCFG_SEL_CHANNEL_9                      (0x00000001UL)
#define _DMAC_CHCFG_SEL_CHANNEL_10                     (0x00000002UL)
#define _DMAC_CHCFG_SEL_CHANNEL_11                     (0x00000003UL)
#define _DMAC_CHCFG_SEL_CHANNEL_12                     (0x00000004UL)
#define _DMAC_CHCFG_SEL_CHANNEL_13                     (0x00000005UL)
#define _DMAC_CHCFG_SEL_CHANNEL_14                     (0x00000006UL)
#define _DMAC_CHCFG_SEL_CHANNEL_15                     (0x00000007UL)
/* DMA Activation Request Source Select (REQD) */
#define _DMAC_CHCFG_REQUEST_BY_SRC_READ                (0x00000000UL)
#define _DMAC_CHCFG_REQUEST_BY_DST_WRITE               (0x00000008UL)
/* L Detection Enable (LOEN) */
#define _DMAC_CHCFG_LOW_LVL_DISABLE                    (0x00000000UL)
#define _DMAC_CHCFG_LOW_LVL_ENABLE                     (0x00000010UL)
#define _DMAC_CHCFG_FALLING_EDGE_DISABLE               (0x00000000UL)
#define _DMAC_CHCFG_FALLING_EDGE_ENABLE                (0x00000010UL)
/* H Detection Enable (HIEN) */
#define _DMAC_CHCFG_HIGH_LVL_DISABLE                   (0x00000000UL)
#define _DMAC_CHCFG_HIGH_LVL_ENABLE                    (0x00000020UL)
#define _DMAC_CHCFG_RISING_EDGE_DISABLE                (0x00000000UL)
#define _DMAC_CHCFG_RISING_EDGE_ENABLE                 (0x00000020UL)
/* Level Detection Enable (LVL) */
#define _DMAC_CHCFG_EDGE_DETECT                        (0x00000000UL)
#define _DMAC_CHCFG_LVL_DETECT                         (0x00000040UL)
/* ACK Mode (AM[2:0]) */
#define _DMAC_CHCFG_ACK_LVL_MODE                       (0x00000100UL)
#define _DMAC_CHCFG_ACK_BUS_MODE                       (0x00000200UL)
#define _DMAC_CHCFG_ACK_MASK                           (0x00000400UL)
/* Descriptor Reload Enable (DRRP) */
#define _DMAC_CHCFG_DES_RELOAD_DISABLE                 (0x00000000UL)
#define _DMAC_CHCFG_DES_RELOAD_ENABLE                  (0x00000800UL)
/* Source Data Size (SDS[2:0]) */
#define _DMAC_CHCFG_SRC_DATA_SZ_8                      (0x00000000UL)
#define _DMAC_CHCFG_SRC_DATA_SZ_16                     (0x00001000UL)
#define _DMAC_CHCFG_SRC_DATA_SZ_32                     (0x00002000UL)
#define _DMAC_CHCFG_SRC_DATA_SZ_128                    (0x00004000UL)
#define _DMAC_CHCFG_SRC_DATA_SZ_256                    (0x00005000UL)
#define _DMAC_CHCFG_SRC_DATA_SZ_512                    (0x00006000UL)
/* Source Data Size (SDS[3]) */
#define _DMAC_CHCFG_SRC_ADDR_NOR                       (0x00000000UL)
#define _DMAC_CHCFG_SRC_ADDR_SKIP                      (0x00008000UL)
/* Destination Data Size (DDS[2:0]) */
#define _DMAC_CHCFG_DST_DATA_SZ_8                      (0x00000000UL)
#define _DMAC_CHCFG_DST_DATA_SZ_16                     (0x00010000UL)
#define _DMAC_CHCFG_DST_DATA_SZ_32                     (0x00020000UL)
#define _DMAC_CHCFG_DST_DATA_SZ_128                    (0x00040000UL)
#define _DMAC_CHCFG_DST_DATA_SZ_256                    (0x00050000UL)
#define _DMAC_CHCFG_DST_DATA_SZ_512                    (0x00060000UL)
/* Destination Data Size (DDS[3]) */
#define _DMAC_CHCFG_DST_ADDR_NOR                       (0x00000000UL)
#define _DMAC_CHCFG_DST_ADDR_SKIP                      (0x00080000UL)
/* Source Address Count Direction (SAD) */
#define _DMAC_CHCFG_SRC_ADDR_INCREMENT                 (0x00000000UL)
#define _DMAC_CHCFG_SRC_ADDR_FIXED                     (0x00100000UL)
/* Destination Address Count Direction (DAD) */
#define _DMAC_CHCFG_DST_ADDR_INCREMENT                 (0x00000000UL)
#define _DMAC_CHCFG_DST_ADDR_FIXED                     (0x00200000UL)
/* Transfer Mode (TM) */
#define _DMAC_CHCFG_SINGLE_TRANSFER                    (0x00000000UL)
#define _DMAC_CHCFG_BLOCK_TRANSER                      (0x00400000UL)
/* Write-Only Mode (WONLY) */
#define _DMAC_CHCFG_WRITE_ONLY_DISABLE                 (0x00000000UL)
#define _DMAC_CHCFG_WRITE_ONLY_ENABLE                  (0x00800000UL)
/* Transfer Completion Interrupt Mask (DEM) */
#define _DMAC_CHCFG_COMPLETE_INT_ENABLE                (0x00000000UL)
#define _DMAC_CHCFG_COMPLETE_INT_MASKED                (0x01000000UL)
/* Descriptor Interrupt Mask (DIM) */
#define _DMAC_CHCFG_DES_COMPLETE_INT_ENABLE            (0x00000000UL)
#define _DMAC_CHCFG_DES_COMPLETE_INT_MASKED            (0x04000000UL)
/* Buffer Flush Enable (SBE) */
#define _DMAC_CHCFG_BUFFER_FLUSH_DISABLE               (0x00000000UL)
#define _DMAC_CHCFG_BUFFER_FLUSH_ENABLE                (0x08000000UL)
/* Next Register Select (RSEL) */
#define _DMAC_CHCFG_NEXT_REGISTER_0                    (0x00000000UL)
#define _DMAC_CHCFG_NEXT_REGISTER_1                    (0x10000000UL)
/* RSEL Reverse (RSW) */
#define _DMAC_CHCFG_RSEL_REVERSE_DISABLE               (0x00000000UL)
#define _DMAC_CHCFG_RSEL_REVERSE_ENABLE                (0x20000000UL)
/* Register Set Enable (REN) */
#define _DMAC_CHCFG_REGISTER_SET_DISABLE               (0x00000000UL)
#define _DMAC_CHCFG_REGISTER_SET_ENABLE                (0x40000000UL)
/* DMA Mode Select (DMS) */
#define _DMAC_CHCFG_REGISTER_MODE                      (0x00000000UL)
#define _DMAC_CHCFG_LINK_MODE                          (0x80000000UL)

/*
    DMA Control Register (DCTRL_X)
*/
/* Priority Control Select (PR) */
#define _DMAC_DCTRL_PRIORITY_FIXED                     (0x00000000UL)
#define _DMAC_DCTRL_PRIORITY_ROUNDROBIN                (0x00000001UL)

/*
    Descriptor header
*/
/* Link valid (LV) */
#define _DMAC_DSC_HEADER_LV_DISABLE                    (0x00000000UL)
#define _DMAC_DSC_HEADER_LV_ENABLE                     (0x01000000UL)
/* Link end (LE) */
#define _DMAC_DSC_HEADER_LINK_CONTINUE                 (0x00000000UL)
#define _DMAC_DSC_HEADER_LINK_END                      (0x02000000UL)
/* Write back disable (WBD) */
#define _DMAC_DSC_HEADER_WBD_VAL_0                     (0x00000000UL)
#define _DMAC_DSC_HEADER_WBD_VAL_1                     (0x04000000UL)
/* Descriptor format (DSCFM[3:0]) */
#define _DMAC_DSC_FORMAT_32_BYTE                       (0x10000000UL)
#define _DMAC_DSC_FORMAT_16_BYTE                       (0x30000000UL)

/*
    Common Control Register (CMNCR)
*/
/* Acknowledge Level (AL0~AL2) */
#define _DMAC_AL0_ACTIVE_LOW      (0x00000000UL) /* Low-active output from DACK0 */
#define _DMAC_AL0_ACTIVE_HIGH     (0x01000000UL) /* High-active output from DACK0 */
#define _DMAC_AL1_ACTIVE_LOW      (0x00000000UL) /* Low-active output from DACK1 */
#define _DMAC_AL1_ACTIVE_HIGH     (0x02000000UL) /* High-active output from DACK1 */
#define _DMAC_AL2_ACTIVE_LOW      (0x00000000UL) /* Low-active output from DACK2 */
#define _DMAC_AL2_ACTIVE_HIGH     (0x04000000UL) /* High-active output from DACK2 */
/* Transfer End Level (TL0~TL2) */
#define _DMAC_TL0_ACTIVE_LOW      (0x00000000UL) /* Low-active output from TEND0 */
#define _DMAC_TL0_ACTIVE_HIGH     (0x10000000UL) /* High-active output from TEND0 */
#define _DMAC_TL1_ACTIVE_LOW      (0x00000000UL) /* Low-active output from TEND1 */
#define _DMAC_TL1_ACTIVE_HIGH     (0x20000000UL) /* High-active output from TEND1 */
#define _DMAC_TL2_ACTIVE_LOW      (0x00000000UL) /* Low-active output from TEND2 */
#define _DMAC_TL2_ACTIVE_HIGH     (0x40000000UL) /* High-active output from TEND1 */

/*
    Interrupt Priority Level Store Register (PRLn)
*/
/* Interrupt Priority Level Store 0 (PRL[3:0]) */
#define _DMAC_PRIORITY_LEVEL0     (0x00000000UL) /* Level 0 (highest) */
#define _DMAC_PRIORITY_LEVEL1     (0x00000001UL) /* Level 1 */
#define _DMAC_PRIORITY_LEVEL2     (0x00000002UL) /* Level 2 */
#define _DMAC_PRIORITY_LEVEL3     (0x00000003UL) /* Level 3 */
#define _DMAC_PRIORITY_LEVEL4     (0x00000004UL) /* Level 4 */
#define _DMAC_PRIORITY_LEVEL5     (0x00000005UL) /* Level 5 */
#define _DMAC_PRIORITY_LEVEL6     (0x00000006UL) /* Level 6 */
#define _DMAC_PRIORITY_LEVEL7     (0x00000007UL) /* Level 7 */
#define _DMAC_PRIORITY_LEVEL8     (0x00000008UL) /* Level 8 */
#define _DMAC_PRIORITY_LEVEL9     (0x00000009UL) /* Level 9 */
#define _DMAC_PRIORITY_LEVEL10    (0x0000000AUL) /* Level 10 */
#define _DMAC_PRIORITY_LEVEL11    (0x0000000BUL) /* Level 11 */
#define _DMAC_PRIORITY_LEVEL12    (0x0000000CUL) /* Level 12 */
#define _DMAC_PRIORITY_LEVEL13    (0x0000000DUL) /* Level 13 */
#define _DMAC_PRIORITY_LEVEL14    (0x0000000EUL) /* Level 14 */
#define _DMAC_PRIORITY_LEVEL15    (0x0000000FUL) /* Level 15 */
#define _DMAC_PRIORITY_LEVEL16    (0x00000000UL) /* Level 16 */
#define _DMAC_PRIORITY_LEVEL17    (0x00000001UL) /* Level 17 */
#define _DMAC_PRIORITY_LEVEL18    (0x00000002UL) /* Level 18 */
#define _DMAC_PRIORITY_LEVEL19    (0x00000003UL) /* Level 19 */
#define _DMAC_PRIORITY_LEVEL20    (0x00000004UL) /* Level 20 */
#define _DMAC_PRIORITY_LEVEL21    (0x00000005UL) /* Level 21 */
#define _DMAC_PRIORITY_LEVEL22    (0x00000006UL) /* Level 22 */
#define _DMAC_PRIORITY_LEVEL23    (0x00000007UL) /* Level 23 */
#define _DMAC_PRIORITY_LEVEL24    (0x00000008UL) /* Level 24 */
#define _DMAC_PRIORITY_LEVEL25    (0x00000009UL) /* Level 25 */
#define _DMAC_PRIORITY_LEVEL26    (0x0000000AUL) /* Level 26 */
#define _DMAC_PRIORITY_LEVEL27    (0x0000000BUL) /* Level 27 */
#define _DMAC_PRIORITY_LEVEL28    (0x0000000CUL) /* Level 28 */
#define _DMAC_PRIORITY_LEVEL29    (0x0000000DUL) /* Level 29 */
#define _DMAC_PRIORITY_LEVEL30    (0x0000000EUL) /* Level 30 */
#define _DMAC_PRIORITY_LEVEL31    (0x0000000FUL) /* Level 31 */

/******************************************DMA********************************************/

#endif
