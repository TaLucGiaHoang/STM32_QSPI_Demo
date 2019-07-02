using System;
using System.IO;
using System.IO.Ports;
using System.Windows.Forms;
using System.Diagnostics;
using System.Security.Cryptography;
using System.Text;

namespace project1
{
    enum STEP
    {
        NONE,
        UPDATE,
        FW_INFO,
        FW_DATA,
        NOTIFY,
        RESET,
        DUMMY,
    }

    public partial class Form1 : Form
    {
        SerialPort myPort;

        public delegate void AddDataDelegate(String myString);
        public AddDataDelegate myDelegate;
        
        public string fileToOpen;
        public long length;
        public int read;
        public string indata;

        private STEP step;
        Stream stream;

        private const string CMD_UPDATE = "u";
        private const string CMD_NOTIFY = "n";
        private const string CMD_RESET = "r";
        private const string CMD_DUMMY = "d";

        public Form1()
        {
            InitializeComponent();            
        }
        private void Form1_Load_1(object sender, EventArgs e)
        {
            //get all serial port
            selectFile.Enabled = false;
            SendFile.Enabled = false;
            dummyCmd.Enabled = false;
            string[] PortList = SerialPort.GetPortNames();
            cbPortNum.Items.Clear();
            if (PortList.Length > 0)
            {
                foreach (string PortName in PortList)
                {
                    cbPortNum.Items.Add(PortName);
                }
                if (cbPortNum.Items.Count > 0)
                {
                    cbPortNum.SelectedIndex = 0;
                }
                cbPortNum.SelectedIndex = 0;
            }
        }
        private void Button1_Click(object sender, EventArgs e)
        {
            //command.Text = "Issue command 1";
            if (String.IsNullOrEmpty(cbPortNum.Text))
            {
                MessageBox.Show("Please choose Port !", "Error");
            }
            else
            {
                try
                {
                    string PortName = cbPortNum.SelectedItem.ToString();
                    myPort = new SerialPort(PortName, Int32.Parse(cbBaudRate.Text), Parity.None, 8, StopBits.One);
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Can not open COM port\n" + ex.ToString(), "Error");
                    return;
                }
                try
                {
                    myPort.Open();                   
                    Button1.Enabled = false;
                    cbPortNum.Enabled = false;
                    cbBaudRate.Enabled = false;
                    selectFile.Enabled = true;
                    dummyCmd.Enabled = true;
                    //auto receive data                     
                    myPort.DataReceived += new SerialDataReceivedEventHandler(mySerialPort_DataReceived);                  
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Can not open COM port\n" + ex.ToString(), "Error");
                }
            }
        }
        //receive data
        delegate void SetTextCallback(string text);

        private void AppendText(string text)
        {
            this.statusCmd.AppendText(text);
        }
        private void SetText(string text)
        {
            if (this.statusCmd.InvokeRequired)
            {
                SetTextCallback d = new SetTextCallback(AppendText);
                this.Invoke(d, new object[] { text });
            }
            else
            {
                this.AppendText(text);
            }
        }

        private  void mySerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;
            if (sp.BytesToRead < 2)
            {
                return;
            }
            indata = sp.ReadExisting();
            SetText(indata.ToString() + Environment.NewLine);
            //statusCmd.Invoke(this.myDelegate, new Object[] { indata });

            switch (step)
            {
                case STEP.UPDATE:                    
                    if (indata == "OK")
                    {
                        stream = File.OpenRead(fileToOpen);
                        length = new System.IO.FileInfo(fileToOpen).Length;                 
                        byte checksum = calculateCheckSum(fileToOpen);
                        SetText("Firmware Size: " + length + Environment.NewLine + "Checksum: " + checksum + Environment.NewLine);
                        myPort.Write(BitConverter.GetBytes(length), 0, 4);
                        myPort.Write(BitConverter.GetBytes(checksum), 0, 1);
                        step = STEP.FW_INFO;
                        SetText("Issue command 2: Firmware Info: ");
                    }
                    break;
             
                case STEP.FW_INFO:
                case STEP.FW_DATA:
                    if (indata == "OK")
                    {
                        byte[] buffer = new byte[512];
                        read = stream.Read(buffer, 0, buffer.Length);
                        if (read > 0)
                        {
                            myPort.Write(buffer, 0, read);
                            SetText("Issue command 3: Firmware data (" + read + ") :");
                            step = STEP.FW_DATA;
                        }
                        else
                        {
                            SetText("Firmware sent" + Environment.NewLine);
                            SetText("Issue command 4: Verify: ");
                            myPort.Write(CMD_NOTIFY);
                            step = STEP.NOTIFY;
                        }
                    }
                    break;
                case STEP.NOTIFY:
                    if (indata == "OK")
                    {
                        SetText("Issue command 5: Reset: ");
                        myPort.Write(CMD_RESET);
                        step = STEP.NONE;

                    }
                    break;
                case STEP.DUMMY:
                    // Response for Dummy Command
                    step = STEP.NONE;
                    break;

            }
            
        }

        //select file to send
        private void SelectFile_Click(object sender, EventArgs e)
        {

            var FD = new OpenFileDialog();
            if (FD.ShowDialog() == DialogResult.OK)
            {
                fileToOpen = FD.FileName;
                //System.IO.StreamReader reader = new System.IO.StreamReader(fileToOpen);//mo file path cách 2
                txtFileName.Text = fileToOpen;
                length = new System.IO.FileInfo(fileToOpen).Length;//size file
                if(length > 2097152)
                {
                    MessageBox.Show("File size exceeds the allowable limit. Can not use");
                }
                             
            }
 
        }
        
        private void SendFile_Click(object sender, EventArgs e)
        {
            step = STEP.UPDATE;

            if (length > 2097152)
            {
                MessageBox.Show("Can not update file. Please select other file");
            }
            else
            {
                try
                {
                    SetText("Issue command 1: Update Request: ");
                    myPort.Write(CMD_UPDATE);
                }
                catch
                {
                    MessageBox.Show("Check connect port");
                }
            }                      
        }

        public  byte[] ReadFully(Stream stream)
        {
            byte[] buffer = new byte[512]; 
            using (MemoryStream ms = new MemoryStream()) 
            {
                read = stream.Read(buffer, 0, buffer.Length);
                if (read <= 0)
                    return ms.ToArray();
                myPort.Write(buffer, 0, read);
                return buffer;
            }
        }

        private void DummyCmd_Click(object sender, EventArgs e)
        {
            step = STEP.DUMMY;
            SetText("Issue dummy command: ");
            myPort.Write(CMD_DUMMY);
        }

        private static byte calculateCheckSum(string fileName)
        {
            Byte[] byteString;
            using (StreamReader file = File.OpenText(fileName))
            {
                string s = file.ReadToEnd();
                byteString = Encoding.UTF8.GetBytes(s);

                Byte chkSumByte = 0x00;
                for (int i = 0; i < byteString.Length; i++)
                    chkSumByte ^= byteString[i];
                return chkSumByte;
            }
        }

        private void TxtFileName_TextChanged(object sender, EventArgs e)
        {
            if (txtFileName.Text != "")
            {
                SendFile.Enabled = true;
            }
            else
            {
                SendFile.Enabled = false;
            }
        }

        private void BtnClear_Click(object sender, EventArgs e)
        {
            statusCmd.Clear();
        }
    }

}
