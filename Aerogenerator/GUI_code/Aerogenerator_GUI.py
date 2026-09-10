from PySide6 import QtWidgets, QtCore
from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import QSyntaxHighlighter, QTextCharFormat, QColor, QFont

# Time series graphing library for PyQt/PySide.
import pyqtgraph as pg

# To find the path of the current script file.
import os.path

# To run external commands in the OS shell.
import subprocess

# The GUI designed in the Qt Designer application.
from main_window import Ui_MainWindow

# The Dialog window designed in the Qt Designer application.
from ESP32_code_dialog import Ui_Dialog_ESP32Code

# Array of data for the charts.
import numpy as np

# Communication agent class
from communication_agent import CommunicationAgent, cmd_messages, ctrl_sys_params, ctrl_var, ctrl_code

# Default updating interval for the charts in [ms].
_charts_update_time_ms = 50

# Updating interval to check for communication test responses in [ms].
_test_comm_response_update_time_ms = 200
_test_comm_response_max_attempts = 5

# Decimation rate for the data in the charts:
_plot_decim_rate = 1

# Syntaxe highlighting for the controller code editor.
class CPlusPlusHighlighter(QSyntaxHighlighter):
    def __init__(self, document):
        super().__init__(document)
        self.rules = []

        # Color Formats
        keyword_format = QTextCharFormat()
        keyword_format.setForeground(QColor("#569CD6"))  # Blue
        keyword_format.setFontWeight(QFont.Bold)

        type_format = QTextCharFormat()
        type_format.setForeground(QColor("#4EC9B0"))  # Teal

        self.comment_format = QTextCharFormat()
        self.comment_format.setForeground(QColor("#6A9955"))  # Green
        self.comment_format.setFontItalic(True)

        string_format = QTextCharFormat()
        string_format.setForeground(QColor("#CE9178"))  # Orange
        
        special_vars_format = QTextCharFormat()
        special_vars_format.setForeground(QColor("#F8FC00"))  # Yellow
        # special_vars_format.setFontWeight(QFont.Bold)

        # 1. Keywords
        keywords = [
            "class", "const", "enum", "explicit", "export", "friend", "inline",
            "namespace", "operator", "private", "protected", "public", "signals",
            "slots", "template", "typename", "using", "virtual", "volatile",
            "return", "if", "else", "switch", "case", "break", "while", "for", "do"
        ]
        for word in keywords:
            pattern = QRegularExpression(f"\\b{word}\\b")
            self.rules.append((pattern, keyword_format))

        # 2. Types
        types = ["char", "double", "float", "int", "long", "short", "signed", "unsigned", "void", "bool"]
        for t in types:
            pattern = QRegularExpression(f"\\b{t}\\b")
            self.rules.append((pattern, type_format))

        # 3. Single-line comments & Strings
        self.rules.append((QRegularExpression("//[^\n]*"), self.comment_format))
        self.rules.append((QRegularExpression("\".*?\""), string_format))

        # 4. Multi-line comment expressions
        self.comment_start_expression = QRegularExpression(r"/\*")
        self.comment_end_expression = QRegularExpression(r"\*/")
        
        # 5. Special variables
        special_vars = ["t","ctrl_last_t", "ref", "xc", "ym", "u"]
        for var in special_vars:
            pattern = QRegularExpression(f"\\b{var}\\b")
            self.rules.append((pattern, special_vars_format))

    def highlightBlock(self, text):
        # Apply standard single-line rules first
        for pattern, fmt in self.rules:
            match_iterator = pattern.globalMatch(text)
            while match_iterator.hasNext():
                match = match_iterator.next()
                self.setFormat(match.capturedStart(), match.capturedLength(), fmt)

        # Handle multi-line comments
        self.setCurrentBlockState(0)
        start_index = 0
        
        # Check if the previous line ended inside a comment
        if self.previousBlockState() == 1:
            start_index = 0
        else:
            match = self.comment_start_expression.match(text)
            start_index = match.capturedStart() if match.hasMatch() else -1

        # Process the current line
        while start_index >= 0:
            end_match = self.comment_end_expression.match(text, start_index)
            end_index = end_match.capturedStart() if end_match.hasMatch() else -1
            comment_length = 0

            if end_index == -1:
                # Comment does not end on this line
                self.setCurrentBlockState(1)
                comment_length = len(text) - start_index
            else:
                # Comment ends on this line
                comment_length = end_index - start_index + end_match.capturedLength()

            self.setFormat(start_index, comment_length, self.comment_format)
            
            # Look for the next comment start if this one ended
            if end_index != -1:
                start_match = self.comment_start_expression.match(text, start_index + comment_length)
                start_index = start_match.capturedStart() if start_match.hasMatch() else -1
            else:
                start_index = -1

class Dialog_ESP32Code(QtWidgets.QDialog, Ui_Dialog_ESP32Code):
    
    def __init__(self,comm_agent):
        super().__init__()
        self.setupUi(self)
        
        # Register the communication agent object.
        if not isinstance(comm_agent,CommunicationAgent):
            raise TypeError("The comm_agent argument must be an instance of the CommunicationAgent class.")           
        self.comm_agent = comm_agent   
        
        self.buttonBox_ok_cancel.setEnabled(False)
        
        self.plainTextEdit_message_compilation.setStyleSheet("background-color: #FFFFFF; color: #000000; font-family: Ubuntu; font-size: 12pt;") 
        self.plainTextEdit_message_upload.setStyleSheet("background-color: #FFFFFF; color: #000000; font-family: Ubuntu; font-size: 12pt;")
        self.label_compilation_status.setText("Compilação do código para o ESP32 ainda não foi realizada.")
        self.label_compilation_status.setStyleSheet("color: gray;")
        self.label_upload_status.setText("Upload do código para o ESP32 ainda não foi realizado.")
        self.label_upload_status.setStyleSheet("color: gray;")
        
        update_timer = QtCore.QTimer(self)
        update_timer.setSingleShot(True)
        update_timer.timeout.connect(self.send_code_to_ESP32)
        update_timer.start(500)
        
    def send_code_to_ESP32(self):        
        
        # Read the CLI_commands.txt file to get the commands to compile and upload the code to the ESP32.
        embedded_code_file = os.path.join(os.path.dirname(__file__), "..", "Aerogenerator_embedded_code", "Aerogenerator_embedded_code.ino")
        try:
            with open(os.path.join(os.path.dirname(__file__), "CLI_commands.txt"), "r") as file:
                CLI_commands = file.read().replace("EMBEDDED_CODE.ino",embedded_code_file).replace("SERIAL_PORT_ESP32", self.comm_agent.get_serial_portname()).splitlines()
        except FileNotFoundError:
            self.plainTextEdit_message_compilation.setPlainText("Erro: Arquivo CLI_commands.txt não encontrado.\n")
            return False

        self.plainTextEdit_message_compilation.setPlainText("Compilando o código para o ESP32...\n")           
        
        # Force redraw of the GUI to show the compilation message before proceeding to compile the code for the ESP32.
        QtCore.QCoreApplication.processEvents()

        try:
            result = subprocess.run(CLI_commands[0],capture_output=True,text=True)
        except Exception as e:
            self.plainTextEdit_message_compilation.appendPlainText(f"Erro ao executar o comando de compilação: {e}\n")
            self.plainTextEdit_message_compilation.setStyleSheet("background-color: #FF9999; color: #000000; font-family: Ubuntu; font-size: 10pt;")
            self.label_compilation_status.setText("Falha na compilação do código para o ESP32.")
            self.label_compilation_status.setStyleSheet("color: red;")
            # Reopen the serial port after uploading the code to the ESP32.
            self.comm_agent.resume_communications()
            return False
            
        if result.returncode == 1:
            # The command failed, so we display the error message in red.
            self.plainTextEdit_message_compilation.appendPlainText(result.stderr)
            self.plainTextEdit_message_compilation.setStyleSheet("background-color: #FF9999; color: #000000; font-family: Ubuntu; font-size: 10pt;")
            self.label_compilation_status.setText("Falha na compilação do código para o ESP32.")
            self.label_compilation_status.setStyleSheet("color: red;")
            # Reopen the serial port after uploading the code to the ESP32.
            self.comm_agent.resume_communications()
            return False
        else:
            # The command succeeded, so we display the output message in green.
            self.plainTextEdit_message_compilation.appendPlainText(result.stdout)
            self.plainTextEdit_message_compilation.setStyleSheet("background-color: #99FF99; color: #000000; font-family: Ubuntu; font-size: 10pt;")
            self.label_compilation_status.setText("Código compilado com sucesso para o ESP32.")
            self.label_compilation_status.setStyleSheet("color: green;")
            
        # Force redraw of the GUI to show the compilation results before proceeding to upload the code to the ESP32.
        QtCore.QCoreApplication.processEvents()
        
        # Close the serial port before uploading the code to the ESP32.
        self.comm_agent.pause_communications()
            
        self.plainTextEdit_message_upload.setPlainText("Enviando o código para o ESP32...\n")           
        
        # Force redraw of the GUI to show the upload message before proceeding to upload the code to the ESP32.
        QtCore.QCoreApplication.processEvents()

        try:
            result = subprocess.run(CLI_commands[1],capture_output=True,text=True)
        except Exception as e:
            self.plainTextEdit_message_upload.appendPlainText(f"Erro ao executar o comando de upload: {e}\n")
            self.plainTextEdit_message_upload.setStyleSheet("background-color: #FF9999; color: #000000; font-family: Ubuntu; font-size: 10pt;")
            self.label_upload_status.setText("Falha ao enviar o código para o ESP32.")
            self.label_upload_status.setStyleSheet("color: red;")
            self.comm_agent.resume_communications()
            return False

        if result.returncode == 1:
            # The command failed, so we display the error message in red.
            self.plainTextEdit_message_upload.appendPlainText(result.stderr)
            self.plainTextEdit_message_upload.setStyleSheet("background-color: #FF9999; color: #000000; font-family: Ubuntu; font-size: 10pt;")
            self.label_upload_status.setText("Falha ao enviar o código para o ESP32.")
            self.label_upload_status.setStyleSheet("color: red;")
            self.comm_agent.resume_communications()
            return False
        else:
            # The command succeeded, so we display the output message in green.
            self.plainTextEdit_message_upload.appendPlainText(result.stdout)
            self.plainTextEdit_message_upload.setStyleSheet("background-color: #99FF99; color: #000000; font-family: Ubuntu; font-size: 10pt;")
            self.label_upload_status.setText("Código enviado com sucesso para o ESP32.")
            self.label_upload_status.setStyleSheet("color: green;")
            
        # Reopen the serial port after uploading the code to the ESP32.
        self.comm_agent.resume_communications()
        
        self.buttonBox_ok_cancel.setEnabled(True)

# Graphical User Interface class where function callbacks are defined 
# and specific settings are done.
class GUIWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    
    def __init__(self,comm_agent,charts_update_time_ms=_charts_update_time_ms,plot_decim_rate=_plot_decim_rate):
        super().__init__()
        self.setupUi(self)
        
        # Register the communication agent object.
        if not isinstance(comm_agent,CommunicationAgent):
            raise TypeError("The comm_agent argument must be an instance of the CommunicationAgent class.")           
        self.comm_agent = comm_agent    

        # Automatic detection of the serial port by default.        
        self.checkBox_auto_serial_port.setChecked(True)
        self.lineEdit_serial_port.setDisabled(True)
        self.lineEdit_serial_port.setText(self.comm_agent.get_serial_portname())
                     
        # Initial experiment configuration
        self.controlled_output = "voltage"
        self.radioButton_voltage.setChecked(True)

        # No dead-zone compensation by default.
        self.groupBox_deadzone_comp.setChecked(False)

        self.test_type = "open loop"
        self.radioButton_open_loop.setChecked(True)
        self.on_radioButton_open_loop_toggled()

        self.reference_input = "manual"
        self.radioButton_manual.setChecked(True)

        # No dead-zone compensation.
        self.groupBox_deadzone_comp.setChecked(False)
        
        # For the custom controller, select an open-loop strategy.
        self.comboBox_ctrl_code.setCurrentIndex(0)
        
        # Prevents editing of the sample time spin box directly using the keyboard.
        self.spinBox_sample_time.findChild(QtWidgets.QLineEdit).setReadOnly(True)
        
        # Charts setup.
        self.configure_GUI_experiment()

        # Setup a timer to periodically update the charts data.
        self.timer_plot = QtCore.QTimer()
        self.timer_plot.setInterval(charts_update_time_ms)
        self.timer_plot.timeout.connect(self.update_plots)

        # Initially, the manual input is zero.
        self.lineEdit_manual_input.setText("0")
        
        # Initialize the PlainTextEdit_ctrl_code area with a syntax highlighter for C++ code.
        self.plainTextEdit_ctrl_code.setStyleSheet("background-color: #444; color: #D4D4D4; font-family: Ubuntu; font-size: 14pt;")
        self.highlighter = CPlusPlusHighlighter(self.plainTextEdit_ctrl_code.document())
        
        
        # This is the manual way of connecting signals and slots.
        # Actions
        self.pushButton_start.clicked.connect(self.start_experiment)
        self.pushButton_stop.clicked.connect(self.stop_experiment)
        self.pushButton_test_comm.clicked.connect(self.communication_test)
        self.commandLinkButton_save_data.clicked.connect(self.save_data)

    def configure_GUI_experiment(self):

        if self.controlled_output == "velocity":
            plot1_trace1 = "Omega [rpm]"
            if self.test_type == "closed loop":
                plot1_trace2 = "Ref. [rpm]"
            plot1_ylabel = "Velocity"
        else:
            plot1_trace1 = "Generated Voltage [mV]"
            plot1_trace2 = "Ref. [mV]"
            plot1_ylabel = "Voltage"

        if self.test_type == "open loop":
            plot2_trace = "PWM input"
            plot2_ylabel = "PWM value"
            self.label_manual_input.setText("PWM =")
            self.horizontalSlider_manual_input.setMaximum(255)
            self.horizontalSlider_manual_input.setMinimum(0)
            self.horizontalSlider_manual_input.setValue(0)
        else:
            plot2_trace = "control action"
            plot2_ylabel = "control action"
            if self.controlled_output == "velocity":
                self.label_manual_input.setText("Referência [rpm] =")
                self.horizontalSlider_manual_input.setMaximum(2000)
                self.horizontalSlider_manual_input.setMinimum(0)
                self.horizontalSlider_manual_input.setValue(0)
                self.horizontalSlider_manual_input.setTickInterval(100)
            else:
                self.label_manual_input.setText("Referência [mV] =")
                self.horizontalSlider_manual_input.setMaximum(2000)
                self.horizontalSlider_manual_input.setMinimum(0)
                self.horizontalSlider_manual_input.setValue(0)
                self.horizontalSlider_manual_input.setTickInterval(100)

        # Plot 1 configuration
        self.widget_plot_1.getPlotItem().clear()
        self.widget_plot_1.hideButtons() # To hide the auto range button.
        self.widget_plot_1.setBackground('w')
        self.widget_plot_1.setTitle("Output")
        self.widget_plot_1.setLabel('left',plot1_ylabel)
        self.widget_plot_1.setLabel('bottom','t (s)')
        self.widget_plot_1.showGrid(x=True, y=True)
        self.widget_plot_1.getPlotItem().addLegend();
        self.widget_plot_1.setXRange(0,self.verticalSlider_TimeWindow.value())
        self.widget_plot_1.setYRange(-2,2)
        self.widget_plot_1.enableAutoRange(axis=pg.ViewBox.YAxis,enable=True)
        # Dashed line example
        # self.pen1 = pg.mkPen(color='b',width=2,style=QtCore.Qt.DashLine)
        pen1 = pg.mkPen(color='b',width=2)
        self.x_data1 = np.array([])
        self.y_data1 = np.array([])
        self.line1 = self.widget_plot_1.plot(self.x_data1,
                                             self.y_data1,
                                             pen=pen1,
                                             name=plot1_trace1)
        pen2 = pg.mkPen(color='g',width=2)
        self.x_data2 = np.array([])
        self.y_data2 = np.array([])
        
        if self.test_type == "closed loop":
            self.line2 = self.widget_plot_1.plot(self.x_data2,
                                                 self.y_data2,
                                                 pen=pen2,
                                                 name=plot1_trace2)

        # Plot 2 configuration.
        self.widget_plot_2.getPlotItem().clear()
        self.widget_plot_2.hideButtons() # To hide the auto range button.
        self.widget_plot_2.setBackground('w')
        self.widget_plot_2.setTitle("Input")
        self.widget_plot_2.setLabel('left',plot2_ylabel)
        self.widget_plot_2.setLabel('bottom','t (s)')
        self.widget_plot_2.showGrid(x=True, y=True)
        self.widget_plot_2.getPlotItem().addLegend();
        self.widget_plot_2.setXRange(0,self.verticalSlider_TimeWindow.value())
        self.widget_plot_2.setYRange(-100,100)
        self.widget_plot_2.enableAutoRange(axis=pg.ViewBox.YAxis,enable=True)
        # Dashed line example
        # self.pen3 = pg.mkPen(color='r',width=2,style=QtCore.Qt.DashLine)
        pen3 = pg.mkPen(color='r',width=2)
        self.x_data3 = np.array([])
        self.y_data3 = np.array([])
        self.line3 = self.widget_plot_2.plot(self.x_data3,
                                             self.y_data3,
                                             pen=pen3,
                                             name=plot2_trace)
        
        # Initial index for the data to be plotted.
        self.data_plot_pos = 0

    def communication_test(self):
        
        # Disable the test button while the test is running.
        self.pushButton_test_comm.setDisabled(True)
        
        # Send test command message.
        self.comm_agent.send_command(cmd_messages['test_comm'])

        # Register the start of the test.
        self.plainTextEdit_message_area.setPlainText("Esperando pela resposta do hardware ESP32...")
        self.plainTextEdit_message_area.repaint()
        
        attempts = 0
        test = False
        while (not test) and (attempts < _test_comm_response_max_attempts):
            test = self.comm_agent.test_comm_ok()
            attempts += 1
            QtCore.QThread.msleep(_test_comm_response_update_time_ms)
            
        if (test == True):
                self.plainTextEdit_message_area.appendPlainText(\
                "\n"\
                "Comunicação com o hardware ESP32 bem sucedida!\n"\
                "Sucesso!")
        else:
            self.plainTextEdit_message_area.appendPlainText(\
                "\n"\
                "Falha de Comunicação: resposta ao teste não foi obtida.")
            
        # Re-enable the test button.
        self.pushButton_test_comm.setDisabled(False)

    def update_serial_port(self):        
    
        if self.checkBox_auto_serial_port.isChecked():
            self.lineEdit_serial_port.setDisabled(True)
            portname = self.comm_agent.get_serial_portname() 
            self.lineEdit_serial_port.setText(portname)
        else:
            # Manually defined port name.    
            portname = self.lineEdit_serial_port.text()
            self.comm_agent.set_serial_portname(portname) 

        # Wait a bit for the communication agent to process the port change.
        QtCore.QThread.msleep(200) 
        
        self.comm_agent.set_serial_portname(portname)    
        self.plainTextEdit_message_area.appendPlainText(f"\nPorta serial definida para '{portname}'.")

    def start_experiment(self):
        
        # Many elements in the GUI will be disabled.
        self.tab_general_configuration.setDisabled(True)
        self.tab_controller.setDisabled(True)
        self.tab_comm_serial.setDisabled(True)
        self.groupBox_SaveData.setDisabled(True)
        self.pushButton_start.setDisabled(True)
        self.pushButton_stop.setDisabled(False)
        
        self.horizontalSlider_manual_input.setValue(0)
        self.lineEdit_manual_input.setText("0") 
        
        # Clear charts data
        self.x_data1 = []
        self.x_data2 = []
        self.x_data3 = []
        
        self.y_data1 = []
        self.y_data2 = []
        self.y_data3 = []
        
        # This is the initial index for the data to be plotted.
        self.data_plot_pos = 0
        
        self.line1.setData(self.x_data1,self.y_data1)
        if self.test_type == "closed loop":
            self.line2.setData(self.x_data2,self.y_data2)
        self.line3.setData(self.x_data3,self.y_data3)
        
        self.line1.clear()
        if self.test_type == "closed loop":
            self.line2.clear()
        self.line3.clear()
        
        self.widget_plot_1.setXRange(0,self.verticalSlider_TimeWindow.value(),padding=0)
        self.widget_plot_2.setXRange(0,self.verticalSlider_TimeWindow.value(),padding=0)
        
        
        # Display the following information in the message area:
        self.plainTextEdit_message_area.setPlainText("Configurando o experimento...")
        
        self.set_deadzone_compensation()
        self.set_controlled_variable()
        self.set_control_code()
        
        # Start the timer that generates a signal to periodically update the charts.
        self.timer_plot.start()

        # Send the start command message.
        self.comm_agent.send_command(cmd_messages["start"])

    def stop_experiment(self):

        # Send the stop command message.
        self.comm_agent.send_command(cmd_messages["stop"])

        # Many elements in the GUI will be disabled.
        self.tab_general_configuration.setDisabled(False)
        self.tab_controller.setDisabled(False)
        self.tab_comm_serial.setDisabled(False)
        self.groupBox_manual_input.setDisabled(False)
        self.groupBox_SaveData.setDisabled(False)
        self.pushButton_start.setDisabled(False)
        self.pushButton_stop.setDisabled(True)

        # Stop the timer that generates a signal to periodically update the charts.
        self.timer_plot.stop()

        # Since the system sets to zero the input when it transitions to the STOPPED state,
        # let's make this clear to the user.
        self.lineEdit_manual_input.setText("0")
        self.horizontalSlider_manual_input.setValue(0)
        
    def set_controlled_variable(self):
        
        if self.radioButton_velocity.isChecked():
            self.plainTextEdit_message_area.appendPlainText('Variável Controlada escolhida: Velocidade angular do Gerador.')
            self.comm_agent.send_command(cmd_messages['set_ctrl_variable'],cmd_val1=ctrl_var['velocity'])
        else:
            self.plainTextEdit_message_area.appendPlainText('Variável Controlada escolhida: Tensão Gerada.')
            self.comm_agent.send_command(cmd_messages['set_ctrl_variable'],cmd_val1=ctrl_var['voltage'])
    
    def set_control_code(self):
        
        if self.radioButton_open_loop.isChecked():
            self.comm_agent.send_command(cmd_messages['set_ctrl_code'],cmd_val1=ctrl_code['ctrl_open_loop'])
            self.plainTextEdit_message_area.appendPlainText("\nControle em Malha Aberta selecionado.")    
        else:
            if self.radioButton_PID_s.isChecked():
                self.plainTextEdit_message_area.appendPlainText("Estratégia de Controle escolhida: PID em 's'.")    
                
                kp,_ = self.process_value_from_text(self.lineEdit_ct_kp.displayText())
                self.plainTextEdit_message_area.appendPlainText(f"Ganho proporcional: kp = {kp}")
                
                if self.groupBox_ct_ti.isChecked():
                    ti,_ = self.process_value_from_text(self.lineEdit_ct_ti.displayText())
                else:
                    ti = 0.0
                self.plainTextEdit_message_area.appendPlainText(f"Tempo Integral: Ti = {ti}")
                
                    
                if self.groupBox_ct_ti.isChecked():
                    td,_ = self.process_value_from_text(self.lineEdit_ct_td.displayText())
                else:
                    td = 0.0
                self.plainTextEdit_message_area.appendPlainText(f"Tempo Derivativo: Td = {td}")
                    
                self.comm_agent.send_command(cmd_messages['set_ctrl_sys_param'],cmd_val1=ctrl_sys_params['ctrl_sys_param_kp'],cmd_val2=kp)
                self.comm_agent.send_command(cmd_messages['set_ctrl_sys_param'],cmd_val1=ctrl_sys_params['ctrl_sys_param_ti'],cmd_val2=ti)
                self.comm_agent.send_command(cmd_messages['set_ctrl_sys_param'],cmd_val1=ctrl_sys_params['ctrl_sys_param_td'],cmd_val2=td)
                self.comm_agent.send_command(cmd_messages['set_ctrl_code'],cmd_val1=ctrl_code['ctrl_pid_ct'])
            
            if self.radioButton_generic.isChecked():
                self.plainTextEdit_message_area.appendPlainText("Estratégia de Controle escolhida: Controlador Genérico (escrito em C++).")    
                self.comm_agent.send_command(cmd_messages['set_ctrl_code'],cmd_val1=ctrl_code['ctrl_custom'])
        
    def send_cmd_manual_input_change(self):

        # Process the text in the Edit line as if it was a mathematical expression,
        # while ignoring letters.
        value, ok = self.process_value_from_text(self.lineEdit_manual_input.displayText())
        if ok:
            self.comm_agent.send_command(cmd_messages['set_ref'],cmd_val2=value)
            
        self.lineEdit_manual_input.setText(str(value))
        
    def process_value_from_text(self, expression):
        # Replace comma with dot to force the usage of dot to represent decimal separator.
        expression = expression.replace(',','.')
        
        # Process the text  as if it was a mathematical expression,
        # while ignoring letters.
        r = ''.join(filter(lambda x: x.isdigit() or x == '.' or x == '-' or x == '+' or x == '*' or x == '/',expression))
        try:
            val = eval(r)
        except:
            return 0.0, False
            
        return val, True
    
    def set_deadzone_compensation(self):
        if self.groupBox_deadzone_comp.isChecked():
            Cn = self.spinBox_Cn.value()
            Cp = self.spinBox_Cp.value()
            self.comm_agent.send_command(cmd_messages['set_deadzone_comp'],cmd_val1=float(Cn),cmd_val2=float(Cp))
            self.plainTextEdit_message_area.appendPlainText(f"Compensação de Zona morta ativada: Cn = {Cn} e Cp = {Cp}.")
        else:
            Cn = 0.0
            Cp = 0.0
            self.comm_agent.send_command(cmd_messages['set_deadzone_comp'],cmd_val1=float(Cn),cmd_val2=float(Cp))
            self.plainTextEdit_message_area.appendPlainText(f"Compensação de Zona morta desabilitada.")
    
    def save_data(self):

        if (self.comm_agent.get_data_count() == 0):
            msgBox = QtWidgets.QErrorMessage(self)
            msgBox.setModal(True)
            msgBox.showMessage("Não há dados para serem salvos no momento.")
        else:
            dialog = QtWidgets.QFileDialog(self)
            dialog.setWindowTitle("Escolha o arquivo de destino:")
            dialog.setFileMode(QtWidgets.QFileDialog.FileMode.AnyFile)
            dialog.setViewMode(QtWidgets.QFileDialog.ViewMode.Detail)
            dialog.setAcceptMode(QtWidgets.QFileDialog.AcceptMode.AcceptSave)
            dialog.setNameFilters({"Arquivos de dados (*.csv)","All files (*)"})
            dialog.setDefaultSuffix(".csv")
            if (dialog.exec()):
                filename = dialog.selectedFiles()
                filename = filename[0]
                
                # Use numpy way of saving text files.
                try:
                    data_array = self.comm_agent.get_data_array()
                    data_count = self.comm_agent.get_data_count()
                    
                    np.savetxt(filename,data_array[0:data_count,:],
                            delimiter=',',fmt="%1.8e",
                            header="Descricao das colunas de dados:\nTempo (s), tensao gerada [mV], velocidade do gerador [rpm], acao de controle, referencia")
                    msgBox = QtWidgets.QErrorMessage(self)
                    msgBox.setModal(True)
                    msgBox.showMessage("Dados salvos! ;-)")
                except:
                    msgBox = QtWidgets.QErrorMessage(self)
                    msgBox.setModal(True)
                    msgBox.showMessage("Dados não foram salvos... :-(")

    def load_ctrl_code_file(self):
        dialog = QtWidgets.QFileDialog(self)
        dialog.setWindowTitle("Escolha o arquivo de código do controlador:")
        dialog.setFileMode(QtWidgets.QFileDialog.FileMode.AnyFile)
        dialog.setViewMode(QtWidgets.QFileDialog.ViewMode.Detail)
        dialog.setAcceptMode(QtWidgets.QFileDialog.AcceptMode.AcceptOpen)
        dialog.setNameFilters({"Arquivos de código C++ (*.cpp *.h *.txt)","All files (*)"})
        if (dialog.exec()):
            filename = dialog.selectedFiles()
            filename = filename[0]
            
            try:
                with open(filename,'r') as ctrl_code_file:
                    try:
                        ctrl_code_text = ctrl_code_file.read()
                        self.plainTextEdit_ctrl_code.setPlainText(ctrl_code_text)
                        self.label_ctrl_code_file_status.setText("Arquivo Carregado: " + os.path.basename(filename))
                        self.label_ctrl_code_file_status.setStyleSheet("color: orange;")
                    except:
                        msgBox = QtWidgets.QErrorMessage(self)
                        msgBox.setModal(True)
                        msgBox.showMessage("Falha ao ler o arquivo de código do controlador.")
            except:
                msgBox = QtWidgets.QErrorMessage(self)
                msgBox.setModal(True)
                msgBox.showMessage("Falha ao abrir o arquivo de código do controlador.")

    def save_ctrl_code_to_file(self):
        
        dialog = QtWidgets.QFileDialog(self)
        dialog.setWindowTitle("Escolha o arquivo de destino para salvar o código do controlador:")
        dialog.setFileMode(QtWidgets.QFileDialog.FileMode.AnyFile)
        dialog.setViewMode(QtWidgets.QFileDialog.ViewMode.Detail)
        dialog.setAcceptMode(QtWidgets.QFileDialog.AcceptMode.AcceptSave)
        dialog.setNameFilters({"Arquivos de código C++ (*.cpp *.h *.txt)","All files (*)"})
        dialog.setDefaultSuffix(".cpp")
        if (dialog.exec()):
            filename = dialog.selectedFiles()
            filename = filename[0]
            
            try:
                with open(filename,'w') as ctrl_code_file:
                    try:
                        ctrl_code_text = self.plainTextEdit_ctrl_code.toPlainText()
                        ctrl_code_file.write(ctrl_code_text)
                        self.label_ctrl_code_file_status.setText("Arquivo Salvo: " + os.path.basename(filename))
                        self.label_ctrl_code_file_status.setStyleSheet("color: green;")
                    except:
                        msgBox = QtWidgets.QErrorMessage(self)
                        msgBox.setModal(True)
                        msgBox.showMessage("Falha ao escrever no arquivo de código do controlador.")
            except:
                msgBox = QtWidgets.QErrorMessage(self)
                msgBox.setModal(True)
                msgBox.showMessage("Falha ao abrir o arquivo de código do controlador para escrita.")

    def send_code_to_ESP32(self):
            # Save the code from the PlainTextEdit_ctrl_code to a temporary file.
            filename = os.path.join(os.path.dirname(__file__),"..","Aerogenerator_embedded_code", "control_strategy.txt")
            print(filename)
            try:
                with open(filename, 'w') as ctrl_code_file:
                    ctrl_code_text = self.plainTextEdit_ctrl_code.toPlainText()
                    ctrl_code_file.write(ctrl_code_text)
            except:
                msgBox = QtWidgets.QErrorMessage(self)
                msgBox.setModal(True)
                msgBox.showMessage("Falha ao salvar o código do controlador para o arquivo \"control_strategy.txt\".")
                return False

            dialog = Dialog_ESP32Code(self.comm_agent)
            if dialog.exec() == 1:
                self.label_ctrl_code_ESP32_status.setText("Código enviado com sucesso para o ESP32.")
                self.label_ctrl_code_ESP32_status.setStyleSheet("color: green;")
                return True
            else:
                self.label_ctrl_code_ESP32_status.setText("Falha ao enviar o código para o ESP32.")
                self.label_ctrl_code_ESP32_status.setStyleSheet("color: red;")
                return False
    
    def update_plots(self):
    
        # Get the current data count.
        data_count = self.comm_agent.get_data_count()
        
        # Update the LCD number of number of collected data points.
        self.lcdNumber_dados_coletados.display(data_count)
        
        # Select the data to be plotted using a decimation rate:
        rows = range(self.data_plot_pos,data_count,_plot_decim_rate)

        if len(rows) > 0:                

            # Retrieve the data from the communication agent.            
            data_array = self.comm_agent.get_data_array(rows)
            
            # All charts have the same time axis data.
            self.x_data1 = np.append(self.x_data1,data_array[:,0])
            self.x_data2 = self.x_data1
            self.x_data3 = self.x_data1

            # Charts Y data depends on the type of experiment:
            if self.controlled_output == "velocity":
                self.y_data1 = np.append(self.y_data1,data_array[:,ctrl_var['velocity']+1])

            if self.controlled_output == "voltage":
                self.y_data1 = np.append(self.y_data1,data_array[:,ctrl_var['voltage']+1]) 

            if self.test_type == "closed loop":
                # Reference signal:    
                self.y_data2 = np.append(self.y_data2,data_array[:,ctrl_var['reference']+1])

            # Control action (plant input signal):
            self.y_data3 = np.append(self.y_data3,data_array[:,ctrl_var['input']+1])

            # Last index used to retrieve data.
            self.data_plot_pos = rows[-1]
            
            self.line1.setData(self.x_data1,self.y_data1)
            if self.test_type == "closed loop":
                self.line2.setData(self.x_data2,self.y_data2)
    
            self.line3.setData(self.x_data3,self.y_data3)
                
        # Sliding Window
        if len(self.x_data1) == 0:
            return
        
        # Inferior limit for the x axis.
        axX = self.widget_plot_1.getAxis('bottom')
        xmax = axX.range[1]
        tspan = self.verticalSlider_TimeWindow.value()
        tmin = self.x_data1[-1] - tspan
        if (xmax <= tmin + tspan):
            self.widget_plot_1.setXRange(tmin,self.x_data1[-1],padding=0)
            self.widget_plot_2.setXRange(tmin,self.x_data1[-1],padding=0)

    # Slots for the widgets signals in the manual input 
    # group (slider, push button, and line edit):
    @QtCore.Slot()
    def on_lineEdit_manual_input_returnPressed(self):
      
        # Process the text in the Edit line as if it was a mathematical expression,
        # while ignoring letters.
        value, ok = self.process_value_from_text(self.lineEdit_manual_input.displayText())
    
        if ok:
            self.lineEdit_manual_input.setText(str(value))
            self.horizontalSlider_manual_input.setValue(value)
        else:
            self.horizontalSlider_manual_input.setValue(0)
            self.lineEdit_manual_input.setText("0")

        self.send_cmd_manual_input_change()

    @QtCore.Slot()
    def on_horizontalSlider_manual_input_sliderPressed(self):        
        self.lineEdit_manual_input.setText(str(self.horizontalSlider_manual_input.value()))
        self.send_cmd_manual_input_change()

    @QtCore.Slot()
    def on_horizontalSlider_manual_input_sliderReleased(self):        
        self.lineEdit_manual_input.setText(str(self.horizontalSlider_manual_input.value()))
        self.send_cmd_manual_input_change()

    @QtCore.Slot()
    def on_pushButton_manual_input_released(self):        
        self.lineEdit_manual_input.setText("0")
        self.horizontalSlider_manual_input.setValue(0)
        self.send_cmd_manual_input_change()

    @QtCore.Slot()
    def on_radioButton_velocity_toggled(self):
        if self.radioButton_velocity.isChecked():
            self.controlled_output = "velocity"
        else:
            self.controlled_output = "voltage"
            
        self.configure_GUI_experiment()

    @QtCore.Slot()
    def on_radioButton_open_loop_toggled(self):
        if self.radioButton_open_loop.isChecked():
            self.test_type = "open loop"
            txtmessage = (
                "Teste em Malha Aberta selecionado.\n"
                "Neste modo de operação, o sistema não utiliza um controlador.\n"
                "O valor do sinal de entrada (PWM) é definido manualmente pelo usuário.\n"
                "Use o controle deslizante ou o campo de edição para definir o valor do PWM."
            )
            self.textEdit_control_info.setPlainText(txtmessage)
        else:
            self.test_type = "closed loop"
            
        self.configure_GUI_experiment()

    @QtCore.Slot()
    def on_radioButton_manual_toggled(self):
        if self.radioButton_manual.isChecked():
            self.reference_input = "manual"
            self.groupBox_manual_input.setDisabled(False)
            self.tableWidget_prog_input.setDisabled(True)
        else:
            self.reference_input = "programmed"
            self.groupBox_manual_input.setDisabled(True)
            self.tableWidget_prog_input.setDisabled(False)
            # Write here the code for sending the programmed reference or input values
            # to the ESP32 embedded code. (September, the 13th, 2025).
            
        self.configure_GUI_experiment()
        
    @QtCore.Slot()
    def on_radioButton_PID_s_toggled(self):
        if self.radioButton_PID_s.isChecked():           
            txtmessage = (
                "Controle PID em 's' selecionado.\n"
                "Neste modo de operação, o sistema utiliza um controlador PID implementado no domínio 's', " 
                "isto é, como se estivéssemos usando um controlador *contínuo*. "
                "Neste caso, é importante usar um intervalo de amostragem suficientemente pequeno.\n\n"
                "A implementação digital real utiliza a transformação bilinear para a ação integral, e o método de Euler"
                " para a ação derivativa.\n\n"
                "\tDefina os parâmetros do controlador (Kp, Ti, Td) nos campos\n"
                "\tapropriados na aba \"PID em 's' \".\n"
            )
            self.textEdit_control_info.setPlainText(txtmessage)
            self.test_type = "closed loop"
    
    @QtCore.Slot()
    def on_radioButton_generic_toggled(self):
        if self.radioButton_generic.isChecked():           
            txtmessage = (
                "Controle genérico selecionado.\n"
                "Neste modo de operação, o sistema utiliza um controlador implementado em C++.\n"
                "O código do controlador pode ser editado diretamente na aba \"Genérico\".\n"
                "Você também pode carregar um arquivo de código existente ou salvar o código atual em um arquivo.\n"
                "Para enviar o código para o ESP32, clique no botão \"Enviar Código para ESP32\"."
            )
            self.textEdit_control_info.setPlainText(txtmessage)
            self.test_type = "closed loop"
    
    @QtCore.Slot()
    def on_pushButton_ctrl_code_released(self):
        ctrl_code = self.comboBox_ctrl_code.currentText()
        match ctrl_code:
            case "Malha Aberta":
                filename = "template_open_loop.txt"
            case "P":
                filename = "template_P_controller.txt"
            case "PI":
                filename = "template_PI_controller.txt"

        filename = os.path.dirname(__file__) + os.path.sep + "ctrl_templates" + os.path.sep + filename
        try:
            with open(filename,'r') as ctrl_code_file:
                try:
                    ctrl_code_text = ctrl_code_file.read()
                except:
                    ctrl_code_text = "// Não consegui carregar o código..."
        except FileNotFoundError:
            ctrl_code_text = f"// Arquivo \"{filename}\" não encontrado..."
            
        self.plainTextEdit_ctrl_code.setPlainText(ctrl_code_text)
        self.label_ctrl_code_file_status.setText("Template Carregado: " + os.path.basename(filename))
        self.label_ctrl_code_file_status.setStyleSheet("color: blue;")

    @QtCore.Slot()
    def on_pushButton_send_code_ESP32_released(self):

        ctrl_code = self.plainTextEdit_ctrl_code.toPlainText()

        filename = os.path.join(os.path.dirname(__file__),"..","Aerogenerator_embedded_code", "control_strategy.txt")
        print(f"Gravando arquivo: {filename}")
        with open(filename,'w') as ctrl_code_file:
            try:
                ctrl_code_file.write(ctrl_code)
            except:
                print("Não consegui gravar o código do controlador...\n")

    @QtCore.Slot()
    def on_checkBox_auto_serial_port_toggled(self):
        if self.checkBox_auto_serial_port.isChecked():
            self.lineEdit_serial_port.setDisabled(True)
        else:
            self.lineEdit_serial_port.setDisabled(False)
            self.lineEdit_serial_port.setText("Porta serial + <ENTER>")
            self.lineEdit_serial_port.setFocus()            
    
    @QtCore.Slot()
    def on_lineEdit_serial_port_returnPressed(self):
        self.update_serial_port()
   
    @QtCore.Slot()
    def on_spinBox_sample_time_valueChanged(self):
        self.checkBox_confirm_ts.setChecked(False)
   
    @QtCore.Slot()
    def on_checkBox_confirm_ts_toggled(self):
        if self.checkBox_confirm_ts.isChecked():
            ts = self.spinBox_sample_time.value()
            self.comm_agent.send_command(cmd_messages['set_ts'],cmd_val1=ts)
            self.plainTextEdit_message_area.appendPlainText(f"\nTempo de amostragem definido para {ts} ms.")
        else:
            pass  
        
    @QtCore.Slot()
    def on_lineEdit_ct_kp_returnPressed(self):
        kp,_ = self.process_value_from_text(self.lineEdit_ct_kp.displayText())    
        self.lineEdit_ct_kp.setText(str(kp)) 
        
    @QtCore.Slot()
    def on_lineEdit_ct_ti_returnPressed(self):
        ti,_ = self.process_value_from_text(self.lineEdit_ct_ti.displayText())
        ti = abs(ti)    
        self.lineEdit_ct_ti.setText(str(ti))
        
    @QtCore.Slot()
    def on_lineEdit_ct_td_returnPressed(self) -> None:
        td,_ = self.process_value_from_text(self.lineEdit_ct_td.displayText())    
        td = abs(td)
        self.lineEdit_ct_kp.setText(str(td))
    
    @QtCore.Slot()
    def on_pushButton_load_ctrl_code_file_released(self) -> None:
        self.load_ctrl_code_file()
        
    @QtCore.Slot()
    def on_pushButton_save_ctrl_code_file_released(self) -> None:
        self.save_ctrl_code_to_file()
        
    @QtCore.Slot()
    def on_plainTextEdit_ctrl_code_textChanged(self) -> None:
        self.label_ctrl_code_file_status.setText("Código do controlador modificado. Lembre-se de salvar!")
        self.label_ctrl_code_file_status.setStyleSheet("color: red;")
        self.label_ctrl_code_ESP32_status.setText("Código do controlador foi modificado. Ainda não enviado para o ESP32.")
        self.label_ctrl_code_ESP32_status.setStyleSheet("color: red;")        
        
    @QtCore.Slot()
    def on_pushButton_send_code_ESP32_released(self) -> None:
        if (self.send_code_to_ESP32()):
            self.label_ctrl_code_ESP32_status.setText("Código do controlador enviado para o ESP32.")
            self.label_ctrl_code_ESP32_status.setStyleSheet("color: blue;")
        else:
            self.label_ctrl_code_ESP32_status.setText("Falha ao enviar o código do controlador para o ESP32.")
            self.label_ctrl_code_ESP32_status.setStyleSheet("color: red;")
            
            
        