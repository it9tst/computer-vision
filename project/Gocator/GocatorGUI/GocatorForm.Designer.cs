
namespace GocatorGUI
{
    partial class GocatorForm
    {
        /// <summary>
        /// Variabile di progettazione necessaria.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Pulire le risorse in uso.
        /// </summary>
        /// <param name="disposing">ha valore true se le risorse gestite devono essere eliminate, false in caso contrario.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Codice generato da Progettazione Windows Form

        /// <summary>
        /// Metodo necessario per il supporto della finestra di progettazione. Non modificare
        /// il contenuto del metodo con l'editor di codice.
        /// </summary>
        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(GocatorForm));
            this.panel1 = new System.Windows.Forms.Panel();
            this.label1 = new System.Windows.Forms.Label();
            this.panel2 = new System.Windows.Forms.Panel();
            this.splitContainer1 = new System.Windows.Forms.SplitContainer();
            this.tabControl1 = new System.Windows.Forms.TabControl();
            this.Output = new System.Windows.Forms.TabPage();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.buttonStopAcquisition = new System.Windows.Forms.Button();
            this.buttonStartAcquisition = new System.Windows.Forms.Button();
            this.panel4 = new System.Windows.Forms.Panel();
            this.textBoxOutput = new System.Windows.Forms.TextBox();
            this.Config = new System.Windows.Forms.TabPage();
            this.label5 = new System.Windows.Forms.Label();
            this.labelConnectExposure = new System.Windows.Forms.Label();
            this.radioButtonType1 = new System.Windows.Forms.RadioButton();
            this.radioButtonType2 = new System.Windows.Forms.RadioButton();
            this.textBoxExposure = new System.Windows.Forms.TextBox();
            this.labelConnectReturn = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.buttonRefresh = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.textBoxSensorIP = new System.Windows.Forms.TextBox();
            this.buttonConnect = new System.Windows.Forms.Button();
            this.panel3 = new System.Windows.Forms.Panel();
            this.buttonStopSensor = new System.Windows.Forms.Button();
            this.buttonStartSensor = new System.Windows.Forms.Button();
            this.buttonLoadPCL = new System.Windows.Forms.Button();
            this.buttonAnalysisOffline = new System.Windows.Forms.Button();
            this.label6 = new System.Windows.Forms.Label();
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.panel1.SuspendLayout();
            this.panel2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).BeginInit();
            this.splitContainer1.Panel2.SuspendLayout();
            this.splitContainer1.SuspendLayout();
            this.tabControl1.SuspendLayout();
            this.Output.SuspendLayout();
            this.tableLayoutPanel1.SuspendLayout();
            this.panel4.SuspendLayout();
            this.Config.SuspendLayout();
            this.panel3.SuspendLayout();
            this.SuspendLayout();
            // 
            // panel1
            // 
            this.panel1.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(51)))), ((int)(((byte)(51)))), ((int)(((byte)(51)))));
            this.panel1.Controls.Add(this.label1);
            this.panel1.Dock = System.Windows.Forms.DockStyle.Top;
            this.panel1.Location = new System.Drawing.Point(0, 0);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(1264, 40);
            this.panel1.TabIndex = 0;
            // 
            // label1
            // 
            this.label1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.label1.Font = new System.Drawing.Font("Roboto Medium", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.Color.GhostWhite;
            this.label1.Location = new System.Drawing.Point(0, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(1264, 40);
            this.label1.TabIndex = 0;
            this.label1.Text = "GOCATOR";
            this.label1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // panel2
            // 
            this.panel2.BackColor = System.Drawing.Color.GhostWhite;
            this.panel2.Controls.Add(this.splitContainer1);
            this.panel2.Controls.Add(this.panel3);
            this.panel2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel2.Location = new System.Drawing.Point(0, 40);
            this.panel2.Name = "panel2";
            this.panel2.Padding = new System.Windows.Forms.Padding(10);
            this.panel2.Size = new System.Drawing.Size(1264, 551);
            this.panel2.TabIndex = 1;
            // 
            // splitContainer1
            // 
            this.splitContainer1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.splitContainer1.Location = new System.Drawing.Point(10, 10);
            this.splitContainer1.Name = "splitContainer1";
            // 
            // splitContainer1.Panel1
            // 
            this.splitContainer1.Panel1.BackColor = System.Drawing.Color.DimGray;
            // 
            // splitContainer1.Panel2
            // 
            this.splitContainer1.Panel2.Controls.Add(this.tabControl1);
            this.splitContainer1.Size = new System.Drawing.Size(1244, 481);
            this.splitContainer1.SplitterDistance = 740;
            this.splitContainer1.TabIndex = 5;
            // 
            // tabControl1
            // 
            this.tabControl1.Controls.Add(this.Output);
            this.tabControl1.Controls.Add(this.Config);
            this.tabControl1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tabControl1.Font = new System.Drawing.Font("Roboto", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.tabControl1.Location = new System.Drawing.Point(0, 0);
            this.tabControl1.Name = "tabControl1";
            this.tabControl1.SelectedIndex = 0;
            this.tabControl1.Size = new System.Drawing.Size(500, 481);
            this.tabControl1.TabIndex = 0;
            // 
            // Output
            // 
            this.Output.Controls.Add(this.tableLayoutPanel1);
            this.Output.Controls.Add(this.panel4);
            this.Output.Location = new System.Drawing.Point(4, 23);
            this.Output.Name = "Output";
            this.Output.Padding = new System.Windows.Forms.Padding(3);
            this.Output.Size = new System.Drawing.Size(492, 454);
            this.Output.TabIndex = 0;
            this.Output.Text = "Output";
            this.Output.UseVisualStyleBackColor = true;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 2;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.Controls.Add(this.buttonStopAcquisition, 1, 0);
            this.tableLayoutPanel1.Controls.Add(this.buttonStartAcquisition, 0, 0);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(3, 401);
            this.tableLayoutPanel1.Margin = new System.Windows.Forms.Padding(0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 1;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(486, 50);
            this.tableLayoutPanel1.TabIndex = 1;
            // 
            // buttonStopAcquisition
            // 
            this.buttonStopAcquisition.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Left | System.Windows.Forms.AnchorStyles.Right)));
            this.buttonStopAcquisition.AutoSize = true;
            this.buttonStopAcquisition.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.buttonStopAcquisition.Font = new System.Drawing.Font("Roboto", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonStopAcquisition.Location = new System.Drawing.Point(243, 13);
            this.buttonStopAcquisition.Margin = new System.Windows.Forms.Padding(0);
            this.buttonStopAcquisition.Name = "buttonStopAcquisition";
            this.buttonStopAcquisition.Size = new System.Drawing.Size(243, 24);
            this.buttonStopAcquisition.TabIndex = 0;
            this.buttonStopAcquisition.Text = "STOP ACQUISITION";
            this.buttonStopAcquisition.UseVisualStyleBackColor = true;
            this.buttonStopAcquisition.Click += new System.EventHandler(this.buttonStopAcquisition_Click);
            // 
            // buttonStartAcquisition
            // 
            this.buttonStartAcquisition.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Left | System.Windows.Forms.AnchorStyles.Right)));
            this.buttonStartAcquisition.AutoSize = true;
            this.buttonStartAcquisition.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.buttonStartAcquisition.Font = new System.Drawing.Font("Roboto", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonStartAcquisition.Location = new System.Drawing.Point(0, 13);
            this.buttonStartAcquisition.Margin = new System.Windows.Forms.Padding(0);
            this.buttonStartAcquisition.Name = "buttonStartAcquisition";
            this.buttonStartAcquisition.Size = new System.Drawing.Size(243, 24);
            this.buttonStartAcquisition.TabIndex = 1;
            this.buttonStartAcquisition.Text = "START ACQUISITION";
            this.buttonStartAcquisition.UseVisualStyleBackColor = true;
            this.buttonStartAcquisition.Click += new System.EventHandler(this.buttonStartAcquisition_Click);
            // 
            // panel4
            // 
            this.panel4.BackColor = System.Drawing.Color.WhiteSmoke;
            this.panel4.Controls.Add(this.textBoxOutput);
            this.panel4.Dock = System.Windows.Forms.DockStyle.Top;
            this.panel4.Location = new System.Drawing.Point(3, 3);
            this.panel4.Name = "panel4";
            this.panel4.Size = new System.Drawing.Size(486, 398);
            this.panel4.TabIndex = 0;
            // 
            // textBoxOutput
            // 
            this.textBoxOutput.BackColor = System.Drawing.Color.Gainsboro;
            this.textBoxOutput.Dock = System.Windows.Forms.DockStyle.Fill;
            this.textBoxOutput.ForeColor = System.Drawing.Color.Black;
            this.textBoxOutput.Location = new System.Drawing.Point(0, 0);
            this.textBoxOutput.Multiline = true;
            this.textBoxOutput.Name = "textBoxOutput";
            this.textBoxOutput.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.textBoxOutput.Size = new System.Drawing.Size(486, 398);
            this.textBoxOutput.TabIndex = 0;
            this.textBoxOutput.Text = resources.GetString("textBoxOutput.Text");
            this.textBoxOutput.TextChanged += new System.EventHandler(this.textBoxOutput_TextChanged);
            // 
            // Config
            // 
            this.Config.BackColor = System.Drawing.Color.White;
            this.Config.Controls.Add(this.label6);
            this.Config.Controls.Add(this.buttonAnalysisOffline);
            this.Config.Controls.Add(this.buttonLoadPCL);
            this.Config.Controls.Add(this.label5);
            this.Config.Controls.Add(this.labelConnectExposure);
            this.Config.Controls.Add(this.radioButtonType1);
            this.Config.Controls.Add(this.radioButtonType2);
            this.Config.Controls.Add(this.textBoxExposure);
            this.Config.Controls.Add(this.labelConnectReturn);
            this.Config.Controls.Add(this.label4);
            this.Config.Controls.Add(this.label3);
            this.Config.Controls.Add(this.buttonRefresh);
            this.Config.Controls.Add(this.label2);
            this.Config.Controls.Add(this.textBoxSensorIP);
            this.Config.Controls.Add(this.buttonConnect);
            this.Config.Location = new System.Drawing.Point(4, 23);
            this.Config.Margin = new System.Windows.Forms.Padding(0);
            this.Config.Name = "Config";
            this.Config.Padding = new System.Windows.Forms.Padding(3);
            this.Config.Size = new System.Drawing.Size(492, 454);
            this.Config.TabIndex = 1;
            this.Config.Text = "Config";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Font = new System.Drawing.Font("Roboto", 9F);
            this.label5.Location = new System.Drawing.Point(10, 220);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(44, 14);
            this.label5.TabIndex = 8;
            this.label5.Text = "Object:";
            // 
            // labelConnectExposure
            // 
            this.labelConnectExposure.AutoSize = true;
            this.labelConnectExposure.Font = new System.Drawing.Font("Roboto", 9F);
            this.labelConnectExposure.ForeColor = System.Drawing.Color.ForestGreen;
            this.labelConnectExposure.Location = new System.Drawing.Point(15, 170);
            this.labelConnectExposure.Name = "labelConnectExposure";
            this.labelConnectExposure.Size = new System.Drawing.Size(141, 14);
            this.labelConnectExposure.TabIndex = 11;
            this.labelConnectExposure.Text = "Ok - New Exposure: 2000";
            this.labelConnectExposure.Visible = false;
            // 
            // radioButtonType1
            // 
            this.radioButtonType1.AutoSize = true;
            this.radioButtonType1.Font = new System.Drawing.Font("Roboto", 9F);
            this.radioButtonType1.Location = new System.Drawing.Point(69, 219);
            this.radioButtonType1.Name = "radioButtonType1";
            this.radioButtonType1.Size = new System.Drawing.Size(58, 18);
            this.radioButtonType1.TabIndex = 6;
            this.radioButtonType1.TabStop = true;
            this.radioButtonType1.Text = "Type1";
            this.radioButtonType1.UseVisualStyleBackColor = true;
            this.radioButtonType1.CheckedChanged += new System.EventHandler(this.radioButtonType1_CheckedChanged);
            // 
            // radioButtonType2
            // 
            this.radioButtonType2.AutoSize = true;
            this.radioButtonType2.Font = new System.Drawing.Font("Roboto", 9F);
            this.radioButtonType2.Location = new System.Drawing.Point(130, 219);
            this.radioButtonType2.Name = "radioButtonType2";
            this.radioButtonType2.Size = new System.Drawing.Size(58, 18);
            this.radioButtonType2.TabIndex = 7;
            this.radioButtonType2.TabStop = true;
            this.radioButtonType2.Text = "Type2";
            this.radioButtonType2.UseVisualStyleBackColor = true;
            this.radioButtonType2.CheckedChanged += new System.EventHandler(this.radioButtonType2_CheckedChanged);
            // 
            // textBoxExposure
            // 
            this.textBoxExposure.Font = new System.Drawing.Font("Roboto", 9F);
            this.textBoxExposure.Location = new System.Drawing.Point(80, 136);
            this.textBoxExposure.Name = "textBoxExposure";
            this.textBoxExposure.Size = new System.Drawing.Size(100, 22);
            this.textBoxExposure.TabIndex = 4;
            this.textBoxExposure.TextChanged += new System.EventHandler(this.textBoxExposure_TextChanged);
            // 
            // labelConnectReturn
            // 
            this.labelConnectReturn.AutoSize = true;
            this.labelConnectReturn.Font = new System.Drawing.Font("Roboto", 9F);
            this.labelConnectReturn.ForeColor = System.Drawing.Color.ForestGreen;
            this.labelConnectReturn.Location = new System.Drawing.Point(15, 90);
            this.labelConnectReturn.Name = "labelConnectReturn";
            this.labelConnectReturn.Size = new System.Drawing.Size(21, 14);
            this.labelConnectReturn.TabIndex = 10;
            this.labelConnectReturn.Text = "Ok";
            this.labelConnectReturn.Visible = false;
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Font = new System.Drawing.Font("Roboto", 9F);
            this.label4.Location = new System.Drawing.Point(10, 140);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(60, 14);
            this.label4.TabIndex = 3;
            this.label4.Text = "Exposure:";
            // 
            // label3
            // 
            this.label3.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            this.label3.AutoSize = true;
            this.label3.Font = new System.Drawing.Font("Roboto", 9F);
            this.label3.Location = new System.Drawing.Point(10, 60);
            this.label3.Margin = new System.Windows.Forms.Padding(0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(61, 14);
            this.label3.TabIndex = 1;
            this.label3.Text = "Sensor IP:";
            // 
            // buttonRefresh
            // 
            this.buttonRefresh.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            this.buttonRefresh.Font = new System.Drawing.Font("Roboto", 9F);
            this.buttonRefresh.Location = new System.Drawing.Point(190, 137);
            this.buttonRefresh.Name = "buttonRefresh";
            this.buttonRefresh.Size = new System.Drawing.Size(75, 19);
            this.buttonRefresh.TabIndex = 9;
            this.buttonRefresh.Text = "REFRESH";
            this.buttonRefresh.UseVisualStyleBackColor = true;
            this.buttonRefresh.Click += new System.EventHandler(this.buttonRefresh_Click);
            // 
            // label2
            // 
            this.label2.Dock = System.Windows.Forms.DockStyle.Top;
            this.label2.Font = new System.Drawing.Font("Roboto", 16F);
            this.label2.Location = new System.Drawing.Point(3, 3);
            this.label2.Margin = new System.Windows.Forms.Padding(0);
            this.label2.Name = "label2";
            this.label2.RightToLeft = System.Windows.Forms.RightToLeft.No;
            this.label2.Size = new System.Drawing.Size(486, 40);
            this.label2.TabIndex = 0;
            this.label2.Text = "Gocator Calibrate";
            this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // textBoxSensorIP
            // 
            this.textBoxSensorIP.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            this.textBoxSensorIP.Font = new System.Drawing.Font("Roboto", 9F);
            this.textBoxSensorIP.Location = new System.Drawing.Point(80, 56);
            this.textBoxSensorIP.Name = "textBoxSensorIP";
            this.textBoxSensorIP.Size = new System.Drawing.Size(100, 22);
            this.textBoxSensorIP.TabIndex = 2;
            this.textBoxSensorIP.TextChanged += new System.EventHandler(this.textBoxSensorIP_TextChanged);
            // 
            // buttonConnect
            // 
            this.buttonConnect.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            this.buttonConnect.Font = new System.Drawing.Font("Roboto", 9F);
            this.buttonConnect.Location = new System.Drawing.Point(190, 57);
            this.buttonConnect.Name = "buttonConnect";
            this.buttonConnect.Size = new System.Drawing.Size(75, 19);
            this.buttonConnect.TabIndex = 5;
            this.buttonConnect.Text = "CONNECT";
            this.buttonConnect.UseVisualStyleBackColor = true;
            this.buttonConnect.Click += new System.EventHandler(this.buttonConnect_Click);
            // 
            // panel3
            // 
            this.panel3.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(51)))), ((int)(((byte)(51)))), ((int)(((byte)(51)))));
            this.panel3.Controls.Add(this.buttonStopSensor);
            this.panel3.Controls.Add(this.buttonStartSensor);
            this.panel3.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.panel3.Location = new System.Drawing.Point(10, 491);
            this.panel3.Name = "panel3";
            this.panel3.Padding = new System.Windows.Forms.Padding(8);
            this.panel3.Size = new System.Drawing.Size(1244, 50);
            this.panel3.TabIndex = 4;
            // 
            // buttonStopSensor
            // 
            this.buttonStopSensor.Dock = System.Windows.Forms.DockStyle.Left;
            this.buttonStopSensor.Font = new System.Drawing.Font("Roboto", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonStopSensor.Location = new System.Drawing.Point(128, 8);
            this.buttonStopSensor.Margin = new System.Windows.Forms.Padding(0);
            this.buttonStopSensor.Name = "buttonStopSensor";
            this.buttonStopSensor.Size = new System.Drawing.Size(120, 34);
            this.buttonStopSensor.TabIndex = 1;
            this.buttonStopSensor.Text = "STOP SENSOR";
            this.buttonStopSensor.UseVisualStyleBackColor = true;
            this.buttonStopSensor.Click += new System.EventHandler(this.buttonStopSensor_Click);
            // 
            // buttonStartSensor
            // 
            this.buttonStartSensor.Dock = System.Windows.Forms.DockStyle.Left;
            this.buttonStartSensor.Font = new System.Drawing.Font("Roboto", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonStartSensor.Location = new System.Drawing.Point(8, 8);
            this.buttonStartSensor.Margin = new System.Windows.Forms.Padding(0);
            this.buttonStartSensor.Name = "buttonStartSensor";
            this.buttonStartSensor.Size = new System.Drawing.Size(120, 34);
            this.buttonStartSensor.TabIndex = 0;
            this.buttonStartSensor.Text = "START SENSOR";
            this.buttonStartSensor.UseVisualStyleBackColor = true;
            this.buttonStartSensor.Click += new System.EventHandler(this.buttonStartSensor_Click);
            // 
            // buttonLoadPCL
            // 
            this.buttonLoadPCL.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            this.buttonLoadPCL.Font = new System.Drawing.Font("Roboto", 9F);
            this.buttonLoadPCL.Location = new System.Drawing.Point(10, 300);
            this.buttonLoadPCL.Name = "buttonLoadPCL";
            this.buttonLoadPCL.Size = new System.Drawing.Size(75, 19);
            this.buttonLoadPCL.TabIndex = 12;
            this.buttonLoadPCL.Text = "LOAD PCL";
            this.buttonLoadPCL.UseVisualStyleBackColor = true;
            this.buttonLoadPCL.Click += new System.EventHandler(this.button1_Click);
            // 
            // buttonAnalysisOffline
            // 
            this.buttonAnalysisOffline.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            this.buttonAnalysisOffline.Font = new System.Drawing.Font("Roboto", 9F);
            this.buttonAnalysisOffline.Location = new System.Drawing.Point(91, 300);
            this.buttonAnalysisOffline.Name = "buttonAnalysisOffline";
            this.buttonAnalysisOffline.Size = new System.Drawing.Size(75, 19);
            this.buttonAnalysisOffline.TabIndex = 13;
            this.buttonAnalysisOffline.Text = "ANALYSIS";
            this.buttonAnalysisOffline.UseVisualStyleBackColor = true;
            this.buttonAnalysisOffline.Click += new System.EventHandler(this.buttonAnalysisOffline_Click);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("Roboto", 9F);
            this.label6.Location = new System.Drawing.Point(10, 280);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(93, 14);
            this.label6.TabIndex = 14;
            this.label6.Text = "Offline Analysis:";
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            this.openFileDialog1.FileOk += new System.ComponentModel.CancelEventHandler(this.openFileDialog1_FileOk);
            // 
            // GocatorForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.SystemColors.Control;
            this.ClientSize = new System.Drawing.Size(1264, 591);
            this.Controls.Add(this.panel2);
            this.Controls.Add(this.panel1);
            this.MinimumSize = new System.Drawing.Size(800, 630);
            this.Name = "GocatorForm";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "Form1";
            this.panel1.ResumeLayout(false);
            this.panel2.ResumeLayout(false);
            this.splitContainer1.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).EndInit();
            this.splitContainer1.ResumeLayout(false);
            this.tabControl1.ResumeLayout(false);
            this.Output.ResumeLayout(false);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.panel4.ResumeLayout(false);
            this.panel4.PerformLayout();
            this.Config.ResumeLayout(false);
            this.Config.PerformLayout();
            this.panel3.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Panel panel2;
        private System.Windows.Forms.Panel panel3;
        private System.Windows.Forms.Button buttonStopSensor;
        private System.Windows.Forms.Button buttonStartSensor;
        private System.Windows.Forms.SplitContainer splitContainer1;
        private System.Windows.Forms.TabControl tabControl1;
        private System.Windows.Forms.TabPage Output;
        private System.Windows.Forms.Button buttonConnect;
        private System.Windows.Forms.TextBox textBoxExposure;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox textBoxSensorIP;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Panel panel4;
        private System.Windows.Forms.RadioButton radioButtonType1;
        private System.Windows.Forms.Button buttonStartAcquisition;
        private System.Windows.Forms.Button buttonStopAcquisition;
        private System.Windows.Forms.RadioButton radioButtonType2;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Button buttonRefresh;
        private System.Windows.Forms.Label labelConnectExposure;
        private System.Windows.Forms.Label labelConnectReturn;
        private System.Windows.Forms.TabPage Config;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.TextBox textBoxOutput;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Button buttonAnalysisOffline;
        private System.Windows.Forms.Button buttonLoadPCL;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
    }
}

