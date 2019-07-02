namespace project1
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.Button1 = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.cbPortNum = new System.Windows.Forms.ComboBox();
            this.label6 = new System.Windows.Forms.Label();
            this.selectFile = new System.Windows.Forms.Button();
            this.SendFile = new System.Windows.Forms.Button();
            this.statusCmd = new System.Windows.Forms.TextBox();
            this.dummyCmd = new System.Windows.Forms.Button();
            this.cbBaudRate = new System.Windows.Forms.ComboBox();
            this.txtFileName = new System.Windows.Forms.TextBox();
            this.btnClear = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // Button1
            // 
            this.Button1.Location = new System.Drawing.Point(368, 15);
            this.Button1.Name = "Button1";
            this.Button1.Size = new System.Drawing.Size(55, 20);
            this.Button1.TabIndex = 2;
            this.Button1.Text = "Connect";
            this.Button1.UseVisualStyleBackColor = true;
            this.Button1.Click += new System.EventHandler(this.Button1_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(18, 18);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(55, 13);
            this.label1.TabIndex = 9;
            this.label1.Text = " COM port";
            // 
            // cbPortNum
            // 
            this.cbPortNum.FormattingEnabled = true;
            this.cbPortNum.Location = new System.Drawing.Point(91, 14);
            this.cbPortNum.Name = "cbPortNum";
            this.cbPortNum.Size = new System.Drawing.Size(145, 21);
            this.cbPortNum.TabIndex = 11;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(21, 51);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(52, 13);
            this.label6.TabIndex = 22;
            this.label6.Text = "File name";
            // 
            // selectFile
            // 
            this.selectFile.Location = new System.Drawing.Point(368, 48);
            this.selectFile.Name = "selectFile";
            this.selectFile.Size = new System.Drawing.Size(55, 20);
            this.selectFile.TabIndex = 26;
            this.selectFile.Text = "Select";
            this.selectFile.UseVisualStyleBackColor = true;
            this.selectFile.Click += new System.EventHandler(this.SelectFile_Click);
            // 
            // SendFile
            // 
            this.SendFile.Location = new System.Drawing.Point(91, 81);
            this.SendFile.Name = "SendFile";
            this.SendFile.Size = new System.Drawing.Size(271, 20);
            this.SendFile.TabIndex = 28;
            this.SendFile.Text = "Start to update firmware";
            this.SendFile.UseVisualStyleBackColor = true;
            this.SendFile.Click += new System.EventHandler(this.SendFile_Click);
            // 
            // statusCmd
            // 
            this.statusCmd.Location = new System.Drawing.Point(91, 115);
            this.statusCmd.Multiline = true;
            this.statusCmd.Name = "statusCmd";
            this.statusCmd.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.statusCmd.Size = new System.Drawing.Size(332, 96);
            this.statusCmd.TabIndex = 30;
            // 
            // dummyCmd
            // 
            this.dummyCmd.Location = new System.Drawing.Point(368, 81);
            this.dummyCmd.Name = "dummyCmd";
            this.dummyCmd.Size = new System.Drawing.Size(55, 20);
            this.dummyCmd.TabIndex = 32;
            this.dummyCmd.Text = "Dummy";
            this.dummyCmd.UseVisualStyleBackColor = true;
            this.dummyCmd.Click += new System.EventHandler(this.DummyCmd_Click);
            // 
            // cbBaudRate
            // 
            this.cbBaudRate.DisplayMember = "9600";
            this.cbBaudRate.FormattingEnabled = true;
            this.cbBaudRate.Items.AddRange(new object[] {
            "9600",
            "38400",
            "115200"});
            this.cbBaudRate.Location = new System.Drawing.Point(253, 14);
            this.cbBaudRate.Name = "cbBaudRate";
            this.cbBaudRate.Size = new System.Drawing.Size(109, 21);
            this.cbBaudRate.TabIndex = 11;
            this.cbBaudRate.Text = "115200";
            // 
            // txtFileName
            // 
            this.txtFileName.Location = new System.Drawing.Point(91, 48);
            this.txtFileName.Name = "txtFileName";
            this.txtFileName.Size = new System.Drawing.Size(271, 20);
            this.txtFileName.TabIndex = 33;
            this.txtFileName.TextChanged += new System.EventHandler(this.TxtFileName_TextChanged);
            // 
            // btnClear
            // 
            this.btnClear.Location = new System.Drawing.Point(91, 217);
            this.btnClear.Name = "btnClear";
            this.btnClear.Size = new System.Drawing.Size(332, 23);
            this.btnClear.TabIndex = 34;
            this.btnClear.Text = "Clear";
            this.btnClear.UseVisualStyleBackColor = true;
            this.btnClear.Click += new System.EventHandler(this.BtnClear_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.SystemColors.ControlLightLight;
            this.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.ClientSize = new System.Drawing.Size(459, 256);
            this.Controls.Add(this.btnClear);
            this.Controls.Add(this.txtFileName);
            this.Controls.Add(this.dummyCmd);
            this.Controls.Add(this.statusCmd);
            this.Controls.Add(this.SendFile);
            this.Controls.Add(this.selectFile);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.cbBaudRate);
            this.Controls.Add(this.cbPortNum);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.Button1);
            this.Name = "Form1";
            this.Text = "COM operation app";
            this.Load += new System.EventHandler(this.Form1_Load_1);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion
        private System.Windows.Forms.Button Button1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ComboBox cbPortNum;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Button selectFile;
        private System.Windows.Forms.Button SendFile;
        private System.Windows.Forms.TextBox statusCmd;
        private System.Windows.Forms.Button dummyCmd;
        private System.Windows.Forms.ComboBox cbBaudRate;
        private System.Windows.Forms.TextBox txtFileName;
        private System.Windows.Forms.Button btnClear;
    }
}

