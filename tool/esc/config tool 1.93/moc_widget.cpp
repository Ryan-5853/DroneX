/****************************************************************************
** Meta object code from reading C++ file 'widget.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.6.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../SerialPortConnector/widget.h"
#include <QtGui/qtextcursor.h>
#include <QtCore/qmetatype.h>

#if __has_include(<QtCore/qtmochelpers.h>)
#include <QtCore/qtmochelpers.h>
#else
QT_BEGIN_MOC_NAMESPACE
#endif


#include <memory>

#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.6.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSWidgetENDCLASS_t {};
static constexpr auto qt_meta_stringdata_CLASSWidgetENDCLASS = QtMocHelpers::stringData(
    "Widget",
    "mspSerialChecksumBuf",
    "uint8_t",
    "",
    "checksum",
    "const uint8_t*",
    "data",
    "len",
    "send_mspCommand",
    "cmd",
    "payload",
    "on_sendButton_clicked",
    "on_disconnectButton_clicked",
    "on_pushButton_clicked",
    "on_sendMessageButton_clicked",
    "on_passthoughButton_clicked",
    "on_horizontalSlider_sliderMoved",
    "position",
    "on_serialSelectorBox_currentTextChanged",
    "arg1",
    "on_pushButton_2_clicked",
    "on_loadBinary_clicked",
    "on_writeBinary_clicked",
    "on_VerifyFlash_clicked",
    "on_initMotor2_clicked",
    "on_initMotor1_clicked",
    "on_initMotor3_clicked",
    "on_initMotor4_clicked",
    "on_writeEEPROM_clicked",
    "on_sendFirstEEPROM_clicked",
    "on_devSettings_stateChanged",
    "on_endPassthrough_clicked",
    "convertFromHex",
    "on_checkBox_stateChanged",
    "on_initMotor1_2_clicked",
    "on_initMotor2_2_clicked",
    "on_initMotor3_2_clicked",
    "on_initMotor4_2_clicked",
    "on_startupPowerSlider_valueChanged",
    "value",
    "on_timingAdvanceSlider_valueChanged",
    "on_pwmFreqSlider_valueChanged",
    "on_varPWMCheckBox_stateChanged",
    "on_m1MSPSlider_valueChanged",
    "on_m2MSPSlider_valueChanged",
    "on_m3MSPSlider_valueChanged",
    "on_m4MSPSlider_valueChanged",
    "on_ConnectedButton_clicked",
    "on_motorKVSlider_valueChanged",
    "on_motorPolesSlider_valueChanged",
    "on_beepVolumeSlider_valueChanged",
    "on_lowThresholdLineEdit_editingFinished",
    "on_servoLowSlider_valueChanged",
    "on_servoHighSlider_valueChanged",
    "on_servoNeutralSlider_valueChanged",
    "on_lowVoltageThresholdSlider_valueChanged",
    "on_highThresholdLineEdit_editingFinished",
    "on_servoNeuralLineEdit_editingFinished",
    "on_servoDeadBandSlider_valueChanged",
    "on_servoDeadbandLineEdit_editingFinished",
    "on_lowVoltageLineEdit_editingFinished",
    "on_writeEEPROM_2_clicked",
    "on_initMotor1_3_clicked",
    "on_initMotor2_3_clicked",
    "on_initMotor3_3_clicked",
    "on_initMotor4_3_clicked",
    "serialInfoStuff",
    "resetESC",
    "on_dragBrakeSlider_valueChanged",
    "on_sineStartupSlider_valueChanged",
    "on_sineModePowerSlider_valueChanged",
    "on_runningBrakeStrength_valueChanged",
    "on_currentSlider_valueChanged",
    "on_temperatureSlider_valueChanged",
    "on_crawler_default_button_clicked",
    "on_saveConfigButton_clicked",
    "on_loadConfigButton_clicked"
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSWidgetENDCLASS_t {
    uint offsetsAndSizes[154];
    char stringdata0[7];
    char stringdata1[21];
    char stringdata2[8];
    char stringdata3[1];
    char stringdata4[9];
    char stringdata5[15];
    char stringdata6[5];
    char stringdata7[4];
    char stringdata8[16];
    char stringdata9[4];
    char stringdata10[8];
    char stringdata11[22];
    char stringdata12[28];
    char stringdata13[22];
    char stringdata14[29];
    char stringdata15[28];
    char stringdata16[32];
    char stringdata17[9];
    char stringdata18[40];
    char stringdata19[5];
    char stringdata20[24];
    char stringdata21[22];
    char stringdata22[23];
    char stringdata23[23];
    char stringdata24[22];
    char stringdata25[22];
    char stringdata26[22];
    char stringdata27[22];
    char stringdata28[23];
    char stringdata29[27];
    char stringdata30[28];
    char stringdata31[26];
    char stringdata32[15];
    char stringdata33[25];
    char stringdata34[24];
    char stringdata35[24];
    char stringdata36[24];
    char stringdata37[24];
    char stringdata38[35];
    char stringdata39[6];
    char stringdata40[36];
    char stringdata41[30];
    char stringdata42[31];
    char stringdata43[28];
    char stringdata44[28];
    char stringdata45[28];
    char stringdata46[28];
    char stringdata47[27];
    char stringdata48[30];
    char stringdata49[33];
    char stringdata50[33];
    char stringdata51[40];
    char stringdata52[31];
    char stringdata53[32];
    char stringdata54[35];
    char stringdata55[42];
    char stringdata56[41];
    char stringdata57[39];
    char stringdata58[36];
    char stringdata59[41];
    char stringdata60[38];
    char stringdata61[25];
    char stringdata62[24];
    char stringdata63[24];
    char stringdata64[24];
    char stringdata65[24];
    char stringdata66[16];
    char stringdata67[9];
    char stringdata68[32];
    char stringdata69[34];
    char stringdata70[36];
    char stringdata71[37];
    char stringdata72[30];
    char stringdata73[34];
    char stringdata74[34];
    char stringdata75[28];
    char stringdata76[28];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSWidgetENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSWidgetENDCLASS_t qt_meta_stringdata_CLASSWidgetENDCLASS = {
    {
        QT_MOC_LITERAL(0, 6),  // "Widget"
        QT_MOC_LITERAL(7, 20),  // "mspSerialChecksumBuf"
        QT_MOC_LITERAL(28, 7),  // "uint8_t"
        QT_MOC_LITERAL(36, 0),  // ""
        QT_MOC_LITERAL(37, 8),  // "checksum"
        QT_MOC_LITERAL(46, 14),  // "const uint8_t*"
        QT_MOC_LITERAL(61, 4),  // "data"
        QT_MOC_LITERAL(66, 3),  // "len"
        QT_MOC_LITERAL(70, 15),  // "send_mspCommand"
        QT_MOC_LITERAL(86, 3),  // "cmd"
        QT_MOC_LITERAL(90, 7),  // "payload"
        QT_MOC_LITERAL(98, 21),  // "on_sendButton_clicked"
        QT_MOC_LITERAL(120, 27),  // "on_disconnectButton_clicked"
        QT_MOC_LITERAL(148, 21),  // "on_pushButton_clicked"
        QT_MOC_LITERAL(170, 28),  // "on_sendMessageButton_clicked"
        QT_MOC_LITERAL(199, 27),  // "on_passthoughButton_clicked"
        QT_MOC_LITERAL(227, 31),  // "on_horizontalSlider_sliderMoved"
        QT_MOC_LITERAL(259, 8),  // "position"
        QT_MOC_LITERAL(268, 39),  // "on_serialSelectorBox_currentT..."
        QT_MOC_LITERAL(308, 4),  // "arg1"
        QT_MOC_LITERAL(313, 23),  // "on_pushButton_2_clicked"
        QT_MOC_LITERAL(337, 21),  // "on_loadBinary_clicked"
        QT_MOC_LITERAL(359, 22),  // "on_writeBinary_clicked"
        QT_MOC_LITERAL(382, 22),  // "on_VerifyFlash_clicked"
        QT_MOC_LITERAL(405, 21),  // "on_initMotor2_clicked"
        QT_MOC_LITERAL(427, 21),  // "on_initMotor1_clicked"
        QT_MOC_LITERAL(449, 21),  // "on_initMotor3_clicked"
        QT_MOC_LITERAL(471, 21),  // "on_initMotor4_clicked"
        QT_MOC_LITERAL(493, 22),  // "on_writeEEPROM_clicked"
        QT_MOC_LITERAL(516, 26),  // "on_sendFirstEEPROM_clicked"
        QT_MOC_LITERAL(543, 27),  // "on_devSettings_stateChanged"
        QT_MOC_LITERAL(571, 25),  // "on_endPassthrough_clicked"
        QT_MOC_LITERAL(597, 14),  // "convertFromHex"
        QT_MOC_LITERAL(612, 24),  // "on_checkBox_stateChanged"
        QT_MOC_LITERAL(637, 23),  // "on_initMotor1_2_clicked"
        QT_MOC_LITERAL(661, 23),  // "on_initMotor2_2_clicked"
        QT_MOC_LITERAL(685, 23),  // "on_initMotor3_2_clicked"
        QT_MOC_LITERAL(709, 23),  // "on_initMotor4_2_clicked"
        QT_MOC_LITERAL(733, 34),  // "on_startupPowerSlider_valueCh..."
        QT_MOC_LITERAL(768, 5),  // "value"
        QT_MOC_LITERAL(774, 35),  // "on_timingAdvanceSlider_valueC..."
        QT_MOC_LITERAL(810, 29),  // "on_pwmFreqSlider_valueChanged"
        QT_MOC_LITERAL(840, 30),  // "on_varPWMCheckBox_stateChanged"
        QT_MOC_LITERAL(871, 27),  // "on_m1MSPSlider_valueChanged"
        QT_MOC_LITERAL(899, 27),  // "on_m2MSPSlider_valueChanged"
        QT_MOC_LITERAL(927, 27),  // "on_m3MSPSlider_valueChanged"
        QT_MOC_LITERAL(955, 27),  // "on_m4MSPSlider_valueChanged"
        QT_MOC_LITERAL(983, 26),  // "on_ConnectedButton_clicked"
        QT_MOC_LITERAL(1010, 29),  // "on_motorKVSlider_valueChanged"
        QT_MOC_LITERAL(1040, 32),  // "on_motorPolesSlider_valueChanged"
        QT_MOC_LITERAL(1073, 32),  // "on_beepVolumeSlider_valueChanged"
        QT_MOC_LITERAL(1106, 39),  // "on_lowThresholdLineEdit_editi..."
        QT_MOC_LITERAL(1146, 30),  // "on_servoLowSlider_valueChanged"
        QT_MOC_LITERAL(1177, 31),  // "on_servoHighSlider_valueChanged"
        QT_MOC_LITERAL(1209, 34),  // "on_servoNeutralSlider_valueCh..."
        QT_MOC_LITERAL(1244, 41),  // "on_lowVoltageThresholdSlider_..."
        QT_MOC_LITERAL(1286, 40),  // "on_highThresholdLineEdit_edit..."
        QT_MOC_LITERAL(1327, 38),  // "on_servoNeuralLineEdit_editin..."
        QT_MOC_LITERAL(1366, 35),  // "on_servoDeadBandSlider_valueC..."
        QT_MOC_LITERAL(1402, 40),  // "on_servoDeadbandLineEdit_edit..."
        QT_MOC_LITERAL(1443, 37),  // "on_lowVoltageLineEdit_editing..."
        QT_MOC_LITERAL(1481, 24),  // "on_writeEEPROM_2_clicked"
        QT_MOC_LITERAL(1506, 23),  // "on_initMotor1_3_clicked"
        QT_MOC_LITERAL(1530, 23),  // "on_initMotor2_3_clicked"
        QT_MOC_LITERAL(1554, 23),  // "on_initMotor3_3_clicked"
        QT_MOC_LITERAL(1578, 23),  // "on_initMotor4_3_clicked"
        QT_MOC_LITERAL(1602, 15),  // "serialInfoStuff"
        QT_MOC_LITERAL(1618, 8),  // "resetESC"
        QT_MOC_LITERAL(1627, 31),  // "on_dragBrakeSlider_valueChanged"
        QT_MOC_LITERAL(1659, 33),  // "on_sineStartupSlider_valueCha..."
        QT_MOC_LITERAL(1693, 35),  // "on_sineModePowerSlider_valueC..."
        QT_MOC_LITERAL(1729, 36),  // "on_runningBrakeStrength_value..."
        QT_MOC_LITERAL(1766, 29),  // "on_currentSlider_valueChanged"
        QT_MOC_LITERAL(1796, 33),  // "on_temperatureSlider_valueCha..."
        QT_MOC_LITERAL(1830, 33),  // "on_crawler_default_button_cli..."
        QT_MOC_LITERAL(1864, 27),  // "on_saveConfigButton_clicked"
        QT_MOC_LITERAL(1892, 27)   // "on_loadConfigButton_clicked"
    },
    "Widget",
    "mspSerialChecksumBuf",
    "uint8_t",
    "",
    "checksum",
    "const uint8_t*",
    "data",
    "len",
    "send_mspCommand",
    "cmd",
    "payload",
    "on_sendButton_clicked",
    "on_disconnectButton_clicked",
    "on_pushButton_clicked",
    "on_sendMessageButton_clicked",
    "on_passthoughButton_clicked",
    "on_horizontalSlider_sliderMoved",
    "position",
    "on_serialSelectorBox_currentTextChanged",
    "arg1",
    "on_pushButton_2_clicked",
    "on_loadBinary_clicked",
    "on_writeBinary_clicked",
    "on_VerifyFlash_clicked",
    "on_initMotor2_clicked",
    "on_initMotor1_clicked",
    "on_initMotor3_clicked",
    "on_initMotor4_clicked",
    "on_writeEEPROM_clicked",
    "on_sendFirstEEPROM_clicked",
    "on_devSettings_stateChanged",
    "on_endPassthrough_clicked",
    "convertFromHex",
    "on_checkBox_stateChanged",
    "on_initMotor1_2_clicked",
    "on_initMotor2_2_clicked",
    "on_initMotor3_2_clicked",
    "on_initMotor4_2_clicked",
    "on_startupPowerSlider_valueChanged",
    "value",
    "on_timingAdvanceSlider_valueChanged",
    "on_pwmFreqSlider_valueChanged",
    "on_varPWMCheckBox_stateChanged",
    "on_m1MSPSlider_valueChanged",
    "on_m2MSPSlider_valueChanged",
    "on_m3MSPSlider_valueChanged",
    "on_m4MSPSlider_valueChanged",
    "on_ConnectedButton_clicked",
    "on_motorKVSlider_valueChanged",
    "on_motorPolesSlider_valueChanged",
    "on_beepVolumeSlider_valueChanged",
    "on_lowThresholdLineEdit_editingFinished",
    "on_servoLowSlider_valueChanged",
    "on_servoHighSlider_valueChanged",
    "on_servoNeutralSlider_valueChanged",
    "on_lowVoltageThresholdSlider_valueChanged",
    "on_highThresholdLineEdit_editingFinished",
    "on_servoNeuralLineEdit_editingFinished",
    "on_servoDeadBandSlider_valueChanged",
    "on_servoDeadbandLineEdit_editingFinished",
    "on_lowVoltageLineEdit_editingFinished",
    "on_writeEEPROM_2_clicked",
    "on_initMotor1_3_clicked",
    "on_initMotor2_3_clicked",
    "on_initMotor3_3_clicked",
    "on_initMotor4_3_clicked",
    "serialInfoStuff",
    "resetESC",
    "on_dragBrakeSlider_valueChanged",
    "on_sineStartupSlider_valueChanged",
    "on_sineModePowerSlider_valueChanged",
    "on_runningBrakeStrength_valueChanged",
    "on_currentSlider_valueChanged",
    "on_temperatureSlider_valueChanged",
    "on_crawler_default_button_clicked",
    "on_saveConfigButton_clicked",
    "on_loadConfigButton_clicked"
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSWidgetENDCLASS[] = {

 // content:
      12,       // revision
       0,       // classname
       0,    0, // classinfo
      65,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    3,  404,    3, 0x08,    1 /* Private */,
       8,    2,  411,    3, 0x08,    5 /* Private */,
      11,    0,  416,    3, 0x08,    8 /* Private */,
      12,    0,  417,    3, 0x08,    9 /* Private */,
      13,    0,  418,    3, 0x08,   10 /* Private */,
      14,    0,  419,    3, 0x08,   11 /* Private */,
      15,    0,  420,    3, 0x08,   12 /* Private */,
      16,    1,  421,    3, 0x08,   13 /* Private */,
      18,    1,  424,    3, 0x08,   15 /* Private */,
      20,    0,  427,    3, 0x08,   17 /* Private */,
      21,    0,  428,    3, 0x08,   18 /* Private */,
      22,    0,  429,    3, 0x08,   19 /* Private */,
      23,    0,  430,    3, 0x08,   20 /* Private */,
      24,    0,  431,    3, 0x08,   21 /* Private */,
      25,    0,  432,    3, 0x08,   22 /* Private */,
      26,    0,  433,    3, 0x08,   23 /* Private */,
      27,    0,  434,    3, 0x08,   24 /* Private */,
      28,    0,  435,    3, 0x08,   25 /* Private */,
      29,    0,  436,    3, 0x08,   26 /* Private */,
      30,    1,  437,    3, 0x08,   27 /* Private */,
      31,    0,  440,    3, 0x08,   29 /* Private */,
      32,    0,  441,    3, 0x08,   30 /* Private */,
      33,    1,  442,    3, 0x08,   31 /* Private */,
      34,    0,  445,    3, 0x08,   33 /* Private */,
      35,    0,  446,    3, 0x08,   34 /* Private */,
      36,    0,  447,    3, 0x08,   35 /* Private */,
      37,    0,  448,    3, 0x08,   36 /* Private */,
      38,    1,  449,    3, 0x08,   37 /* Private */,
      40,    1,  452,    3, 0x08,   39 /* Private */,
      41,    1,  455,    3, 0x08,   41 /* Private */,
      42,    1,  458,    3, 0x08,   43 /* Private */,
      43,    1,  461,    3, 0x08,   45 /* Private */,
      44,    1,  464,    3, 0x08,   47 /* Private */,
      45,    1,  467,    3, 0x08,   49 /* Private */,
      46,    1,  470,    3, 0x08,   51 /* Private */,
      47,    0,  473,    3, 0x08,   53 /* Private */,
      48,    1,  474,    3, 0x08,   54 /* Private */,
      49,    1,  477,    3, 0x08,   56 /* Private */,
      50,    1,  480,    3, 0x08,   58 /* Private */,
      51,    0,  483,    3, 0x08,   60 /* Private */,
      52,    1,  484,    3, 0x08,   61 /* Private */,
      53,    1,  487,    3, 0x08,   63 /* Private */,
      54,    1,  490,    3, 0x08,   65 /* Private */,
      55,    1,  493,    3, 0x08,   67 /* Private */,
      56,    0,  496,    3, 0x08,   69 /* Private */,
      57,    0,  497,    3, 0x08,   70 /* Private */,
      58,    1,  498,    3, 0x08,   71 /* Private */,
      59,    0,  501,    3, 0x08,   73 /* Private */,
      60,    0,  502,    3, 0x08,   74 /* Private */,
      61,    0,  503,    3, 0x08,   75 /* Private */,
      62,    0,  504,    3, 0x08,   76 /* Private */,
      63,    0,  505,    3, 0x08,   77 /* Private */,
      64,    0,  506,    3, 0x08,   78 /* Private */,
      65,    0,  507,    3, 0x08,   79 /* Private */,
      66,    0,  508,    3, 0x08,   80 /* Private */,
      67,    0,  509,    3, 0x08,   81 /* Private */,
      68,    1,  510,    3, 0x08,   82 /* Private */,
      69,    1,  513,    3, 0x08,   84 /* Private */,
      70,    1,  516,    3, 0x08,   86 /* Private */,
      71,    1,  519,    3, 0x08,   88 /* Private */,
      72,    1,  522,    3, 0x08,   90 /* Private */,
      73,    1,  525,    3, 0x08,   92 /* Private */,
      74,    0,  528,    3, 0x08,   94 /* Private */,
      75,    0,  529,    3, 0x08,   95 /* Private */,
      76,    0,  530,    3, 0x08,   96 /* Private */,

 // slots: parameters
    0x80000000 | 2, 0x80000000 | 2, 0x80000000 | 5, QMetaType::Int,    4,    6,    7,
    QMetaType::Void, 0x80000000 | 2, QMetaType::QByteArray,    9,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::QString,   19,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   19,
    QMetaType::Void,
    QMetaType::QByteArray,
    QMetaType::Void, QMetaType::Int,   19,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   19,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void, QMetaType::Int,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject Widget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_CLASSWidgetENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSWidgetENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSWidgetENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<Widget, std::true_type>,
        // method 'mspSerialChecksumBuf'
        QtPrivate::TypeAndForceComplete<uint8_t, std::false_type>,
        QtPrivate::TypeAndForceComplete<uint8_t, std::false_type>,
        QtPrivate::TypeAndForceComplete<const uint8_t *, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'send_mspCommand'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<uint8_t, std::false_type>,
        QtPrivate::TypeAndForceComplete<QByteArray, std::false_type>,
        // method 'on_sendButton_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_disconnectButton_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_pushButton_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_sendMessageButton_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_passthoughButton_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_horizontalSlider_sliderMoved'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_serialSelectorBox_currentTextChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'on_pushButton_2_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_loadBinary_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_writeBinary_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_VerifyFlash_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_initMotor2_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_initMotor1_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_initMotor3_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_initMotor4_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_writeEEPROM_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_sendFirstEEPROM_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_devSettings_stateChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_endPassthrough_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'convertFromHex'
        QtPrivate::TypeAndForceComplete<QByteArray, std::false_type>,
        // method 'on_checkBox_stateChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_initMotor1_2_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_initMotor2_2_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_initMotor3_2_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_initMotor4_2_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_startupPowerSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_timingAdvanceSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_pwmFreqSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_varPWMCheckBox_stateChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_m1MSPSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_m2MSPSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_m3MSPSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_m4MSPSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_ConnectedButton_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_motorKVSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_motorPolesSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_beepVolumeSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_lowThresholdLineEdit_editingFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_servoLowSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_servoHighSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_servoNeutralSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_lowVoltageThresholdSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_highThresholdLineEdit_editingFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_servoNeuralLineEdit_editingFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_servoDeadBandSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_servoDeadbandLineEdit_editingFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_lowVoltageLineEdit_editingFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_writeEEPROM_2_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_initMotor1_3_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_initMotor2_3_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_initMotor3_3_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_initMotor4_3_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'serialInfoStuff'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'resetESC'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_dragBrakeSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_sineStartupSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_sineModePowerSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_runningBrakeStrength_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_currentSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_temperatureSlider_valueChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'on_crawler_default_button_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_saveConfigButton_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_loadConfigButton_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void Widget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Widget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: { uint8_t _r = _t->mspSerialChecksumBuf((*reinterpret_cast< std::add_pointer_t<uint8_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<const uint8_t*>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[3])));
            if (_a[0]) *reinterpret_cast< uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 1: _t->send_mspCommand((*reinterpret_cast< std::add_pointer_t<uint8_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QByteArray>>(_a[2]))); break;
        case 2: _t->on_sendButton_clicked(); break;
        case 3: _t->on_disconnectButton_clicked(); break;
        case 4: _t->on_pushButton_clicked(); break;
        case 5: _t->on_sendMessageButton_clicked(); break;
        case 6: _t->on_passthoughButton_clicked(); break;
        case 7: _t->on_horizontalSlider_sliderMoved((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 8: _t->on_serialSelectorBox_currentTextChanged((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 9: _t->on_pushButton_2_clicked(); break;
        case 10: _t->on_loadBinary_clicked(); break;
        case 11: _t->on_writeBinary_clicked(); break;
        case 12: _t->on_VerifyFlash_clicked(); break;
        case 13: _t->on_initMotor2_clicked(); break;
        case 14: _t->on_initMotor1_clicked(); break;
        case 15: _t->on_initMotor3_clicked(); break;
        case 16: _t->on_initMotor4_clicked(); break;
        case 17: _t->on_writeEEPROM_clicked(); break;
        case 18: _t->on_sendFirstEEPROM_clicked(); break;
        case 19: _t->on_devSettings_stateChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 20: _t->on_endPassthrough_clicked(); break;
        case 21: { QByteArray _r = _t->convertFromHex();
            if (_a[0]) *reinterpret_cast< QByteArray*>(_a[0]) = std::move(_r); }  break;
        case 22: _t->on_checkBox_stateChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 23: _t->on_initMotor1_2_clicked(); break;
        case 24: _t->on_initMotor2_2_clicked(); break;
        case 25: _t->on_initMotor3_2_clicked(); break;
        case 26: _t->on_initMotor4_2_clicked(); break;
        case 27: _t->on_startupPowerSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 28: _t->on_timingAdvanceSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 29: _t->on_pwmFreqSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 30: _t->on_varPWMCheckBox_stateChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 31: _t->on_m1MSPSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 32: _t->on_m2MSPSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 33: _t->on_m3MSPSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 34: _t->on_m4MSPSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 35: _t->on_ConnectedButton_clicked(); break;
        case 36: _t->on_motorKVSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 37: _t->on_motorPolesSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 38: _t->on_beepVolumeSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 39: _t->on_lowThresholdLineEdit_editingFinished(); break;
        case 40: _t->on_servoLowSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 41: _t->on_servoHighSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 42: _t->on_servoNeutralSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 43: _t->on_lowVoltageThresholdSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 44: _t->on_highThresholdLineEdit_editingFinished(); break;
        case 45: _t->on_servoNeuralLineEdit_editingFinished(); break;
        case 46: _t->on_servoDeadBandSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 47: _t->on_servoDeadbandLineEdit_editingFinished(); break;
        case 48: _t->on_lowVoltageLineEdit_editingFinished(); break;
        case 49: _t->on_writeEEPROM_2_clicked(); break;
        case 50: _t->on_initMotor1_3_clicked(); break;
        case 51: _t->on_initMotor2_3_clicked(); break;
        case 52: _t->on_initMotor3_3_clicked(); break;
        case 53: _t->on_initMotor4_3_clicked(); break;
        case 54: _t->serialInfoStuff(); break;
        case 55: _t->resetESC(); break;
        case 56: _t->on_dragBrakeSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 57: _t->on_sineStartupSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 58: _t->on_sineModePowerSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 59: _t->on_runningBrakeStrength_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 60: _t->on_currentSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 61: _t->on_temperatureSlider_valueChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 62: _t->on_crawler_default_button_clicked(); break;
        case 63: _t->on_saveConfigButton_clicked(); break;
        case 64: _t->on_loadConfigButton_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject *Widget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Widget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSWidgetENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int Widget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 65)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 65;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 65)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 65;
    }
    return _id;
}
QT_WARNING_POP
