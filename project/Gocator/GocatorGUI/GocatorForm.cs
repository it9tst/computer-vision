using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace GocatorGUI {
    public partial class GocatorForm : Form {

        private CWrapper wrapper;
        private string strfilename;

        public GocatorForm() {
            InitializeComponent();
            wrapper = new CWrapper();
        }

        private void buttonStartSensor_Click(object sender, EventArgs e) {
            
        }

        private void buttonStopSensor_Click(object sender, EventArgs e) {

        }

        

        // Panel Output
        private void textBoxOutput_TextChanged(object sender, EventArgs e) {

        }

        private void buttonStartAcquisition_Click(object sender, EventArgs e) {

        }

        private void buttonStopAcquisition_Click(object sender, EventArgs e) {

        }



        // Panel Config
        private void textBoxSensorIP_TextChanged(object sender, EventArgs e) {

        }

        private void buttonConnect_Click(object sender, EventArgs e) {

            wrapper.GocatorManager_SetParameter();
            wrapper.GocatorManager_Init();
        }

        private void textBoxExposure_TextChanged(object sender, EventArgs e) {

        }

        private void buttonRefresh_Click(object sender, EventArgs e) {

        }

        private void radioButtonType1_CheckedChanged(object sender, EventArgs e) {

        }

        private void radioButtonType2_CheckedChanged(object sender, EventArgs e) {

        }

        private void button1_Click(object sender, EventArgs e) {

            OpenFileDialog openFileDialog1 = new OpenFileDialog();

            if (openFileDialog1.ShowDialog() == System.Windows.Forms.DialogResult.OK) {
                strfilename = openFileDialog1.FileName;
                wrapper.GocatorManager_LoadPointCloud(strfilename);
            }
        }

        private void openFileDialog1_FileOk(object sender, CancelEventArgs e) {

        }

        private void buttonAnalysisOffline_Click(object sender, EventArgs e) {
            wrapper.GocatorManager_OfflineAnalysis();
        }
    }
}
