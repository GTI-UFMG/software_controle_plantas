# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'ESP32_code_dialogiAwarP.ui'
##
## Created by: Qt User Interface Compiler version 6.9.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QAbstractButton, QApplication, QDialog, QDialogButtonBox,
    QFrame, QLabel, QPlainTextEdit, QSizePolicy,
    QVBoxLayout, QWidget)

class Ui_Dialog_ESP32Code(object):
    def setupUi(self, Dialog_ESP32Code):
        if not Dialog_ESP32Code.objectName():
            Dialog_ESP32Code.setObjectName(u"Dialog_ESP32Code")
        Dialog_ESP32Code.setWindowModality(Qt.WindowModality.ApplicationModal)
        Dialog_ESP32Code.setEnabled(True)
        Dialog_ESP32Code.resize(640, 386)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Dialog_ESP32Code.sizePolicy().hasHeightForWidth())
        Dialog_ESP32Code.setSizePolicy(sizePolicy)
        Dialog_ESP32Code.setMinimumSize(QSize(640, 256))
        font = QFont()
        font.setStrikeOut(False)
        Dialog_ESP32Code.setFont(font)
        icon = QIcon(QIcon.fromTheme(QIcon.ThemeIcon.MediaSeekForward))
        Dialog_ESP32Code.setWindowIcon(icon)
        Dialog_ESP32Code.setSizeGripEnabled(True)
        Dialog_ESP32Code.setModal(True)
        self.verticalLayout = QVBoxLayout(Dialog_ESP32Code)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label_compilation_status = QLabel(Dialog_ESP32Code)
        self.label_compilation_status.setObjectName(u"label_compilation_status")

        self.verticalLayout.addWidget(self.label_compilation_status)

        self.plainTextEdit_message_compilation = QPlainTextEdit(Dialog_ESP32Code)
        self.plainTextEdit_message_compilation.setObjectName(u"plainTextEdit_message_compilation")
        self.plainTextEdit_message_compilation.setStyleSheet(u"background-color: rgb(255, 255, 255);")
        self.plainTextEdit_message_compilation.setFrameShape(QFrame.Shape.Box)
        self.plainTextEdit_message_compilation.setFrameShadow(QFrame.Shadow.Plain)
        self.plainTextEdit_message_compilation.setLineWidth(1)

        self.verticalLayout.addWidget(self.plainTextEdit_message_compilation)

        self.label_upload_status = QLabel(Dialog_ESP32Code)
        self.label_upload_status.setObjectName(u"label_upload_status")

        self.verticalLayout.addWidget(self.label_upload_status)

        self.plainTextEdit_message_upload = QPlainTextEdit(Dialog_ESP32Code)
        self.plainTextEdit_message_upload.setObjectName(u"plainTextEdit_message_upload")
        self.plainTextEdit_message_upload.setStyleSheet(u"background-color: rgb(255, 255, 255);")
        self.plainTextEdit_message_upload.setFrameShape(QFrame.Shape.Box)
        self.plainTextEdit_message_upload.setFrameShadow(QFrame.Shadow.Plain)

        self.verticalLayout.addWidget(self.plainTextEdit_message_upload)

        self.buttonBox_ok_cancel = QDialogButtonBox(Dialog_ESP32Code)
        self.buttonBox_ok_cancel.setObjectName(u"buttonBox_ok_cancel")
        self.buttonBox_ok_cancel.setEnabled(False)
        self.buttonBox_ok_cancel.setStandardButtons(QDialogButtonBox.StandardButton.Cancel|QDialogButtonBox.StandardButton.Ok)

        self.verticalLayout.addWidget(self.buttonBox_ok_cancel)


        self.retranslateUi(Dialog_ESP32Code)
        self.buttonBox_ok_cancel.rejected.connect(Dialog_ESP32Code.close)
        self.buttonBox_ok_cancel.accepted.connect(Dialog_ESP32Code.accept)

        QMetaObject.connectSlotsByName(Dialog_ESP32Code)
    # setupUi

    def retranslateUi(self, Dialog_ESP32Code):
        Dialog_ESP32Code.setWindowTitle(QCoreApplication.translate("Dialog_ESP32Code", u"Compila\u00e7\u00e3o e Envio de C\u00f3digo para o ESP32", None))
#if QT_CONFIG(tooltip)
        Dialog_ESP32Code.setToolTip(QCoreApplication.translate("Dialog_ESP32Code", u"Janela que mostra informa\u00e7\u00e3o sobre o processo de compila\u00e7\u00e3o e envio de c\u00f3digo para o ESP32.", None))
#endif // QT_CONFIG(tooltip)
        self.label_compilation_status.setText(QCoreApplication.translate("Dialog_ESP32Code", u"Status da Compila\u00e7\u00e3o.", None))
        self.label_upload_status.setText(QCoreApplication.translate("Dialog_ESP32Code", u"Status do Envio para o ESP32.", None))
    # retranslateUi

