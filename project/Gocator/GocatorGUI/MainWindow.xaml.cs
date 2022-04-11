using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Microsoft.Win32;

namespace GocatorGUI {
    /// <summary>
    /// Logica di interazione per MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window {

        private CWrapper wrapper = new CWrapper();
        private CClient client = new CClient();
        private int STRING_MAX_LENGTH = 5000;
        private string strfilename;
        private int type = 1;
        private bool checkSavePCD = false;

        public MainWindow() {
            this.InitializeComponent();

            wrapper.GocatorManager_ServerStart();

            Thread thread = new Thread(delegate () {
                client.StartClient(this);
            });
            thread.Start();
        }

        public void CreateModel(List<Point3D> points) {
            
            Point3DCollection dataList = new Point3DCollection();
            PointsVisual3D cloudPoints = new PointsVisual3D { Color = Colors.White, Size = 1.0f };
            foreach (Point3D p in points) {
                dataList.Add(p);
            }
            cloudPoints.Points = dataList;

            //List<PointsVisual3D> numberList = new List<PointsVisual3D>() {  };
            //numberList.Add(cloudPoints);

            // Add geometry to helixPlot. It renders asynchronously in the WPF composite render thread...
            helixPlot.Children.Add(cloudPoints);
        }

        private void Border_MouseDown(object sender, MouseButtonEventArgs e) {
            if (e.LeftButton == MouseButtonState.Pressed) {
                DragMove();
            }
        }

        private void ButtonMinimize_Click(object sender, RoutedEventArgs e) {
            Application.Current.MainWindow.WindowState = WindowState.Minimized;
        }

        private void ButtonWindowState_Click(object sender, RoutedEventArgs e) {
            if (Application.Current.MainWindow.WindowState != WindowState.Maximized) {
                Application.Current.MainWindow.WindowState = WindowState.Maximized;
            } else {
                Application.Current.MainWindow.WindowState = WindowState.Normal;
            }
        }

        private void ButtonClose_Click(object sender, RoutedEventArgs e) {
            Application.Current.Shutdown();
        }

        // Panel Config
        private void buttonConnect_Click(object sender, RoutedEventArgs e) {
            
            StringBuilder message = new StringBuilder(STRING_MAX_LENGTH);
            string sensor_ip = textBoxSensorIP.Text;

            wrapper.GocatorManager_SetParameter(message, STRING_MAX_LENGTH, sensor_ip, 1);
            wrapper.GocatorManager_Init(message, STRING_MAX_LENGTH);

            if (!message.ToString().Equals("OK")) {
                labelConnect.Foreground = new SolidColorBrush(Colors.Red);
            }
            labelConnect.Content = message.ToString().Replace('-', '\n');
            labelConnect.Visibility = Visibility.Visible;
        }

        private void buttonRefresh_Click(object sender, RoutedEventArgs e) {

            StringBuilder message = new StringBuilder(STRING_MAX_LENGTH);
            string exposure = textBoxExposure.Text;

            wrapper.GocatorManager_SetParameter(message, STRING_MAX_LENGTH, exposure, 2);

            if (!message.ToString().Equals("OK")) {
                labelRefresh.Foreground = new SolidColorBrush(Colors.Red);
            }
            labelRefresh.Content = message.ToString().Replace('-', '\n');
            labelRefresh.Visibility = Visibility.Visible;
        }

        private void buttonLoadPCL_Click(object sender, RoutedEventArgs e) {

            StringBuilder message = new StringBuilder(STRING_MAX_LENGTH);

            OpenFileDialog openFileDialog = new OpenFileDialog();
            
            if (openFileDialog.ShowDialog() == true) {
                
                strfilename = openFileDialog.FileName;
                wrapper.GocatorManager_LoadPointCloud(message, STRING_MAX_LENGTH, strfilename);

                if (!message.ToString().Equals("OK")) {
                    labelLoadPCL.Foreground = new SolidColorBrush(Colors.Red);
                }
                labelLoadPCL.Content = message.ToString().Replace('-', '\n');
                labelLoadPCL.Visibility = Visibility.Visible;
            }
        }

        // Button on Bottom
        private void buttonBackward_Click(object sender, RoutedEventArgs e) {
            // scorri point cloud
        }

        private void buttonForward_Click(object sender, RoutedEventArgs e) {
            // scorri point cloud
        }

        private void buttonStartAcquisition_Click(object sender, RoutedEventArgs e) {

            StringBuilder message = new StringBuilder(STRING_MAX_LENGTH);

            checkInit();
            wrapper.GocatorManager_StartAcquisition(message, STRING_MAX_LENGTH, type, checkSavePCD);

            if (!message.ToString().Equals("OK")) {
                MessageBox.Show(message.ToString());
            }
        }

        private void buttonStopAcquisition_Click(object sender, RoutedEventArgs e) {

            StringBuilder message = new StringBuilder(STRING_MAX_LENGTH);

            wrapper.GocatorManager_StopAcquisition(message, STRING_MAX_LENGTH);

            if (!message.ToString().Equals("OK")) {
                MessageBox.Show(message.ToString());
            }
        }

        private void buttonFileAnalysis_Click(object sender, RoutedEventArgs e) {

            checkInit();
            wrapper.GocatorManager_FileAnalysis(type, checkSavePCD);
        }

        private void checkInit() {

            if (radioType1.IsChecked == true) {
                type = 1;
            } else {
                type = 2;
            }

            if (checkSaveFilePCD.IsChecked == true) {
                checkSavePCD = true;
            } else {
                checkSavePCD = false;
            }
        }
    }
}
