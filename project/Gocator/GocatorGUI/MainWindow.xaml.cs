using System;
using System.Collections.Generic;
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

        private CWrapper wrapper;
        private CClient client;
        private string strfilename;
        private int type = 1;
        private bool checkSavePCL = false;

        public MainWindow() {
            this.InitializeComponent();

            wrapper = new CWrapper();
            wrapper.GocatorManager_ServerStart();
            client = new CClient();

            Thread thread = new Thread(client.StartClient);
            thread.Start();
            
            CreateModel();
        }

        public void CreateModel() {
            List<Point3D> points;
            points = new List<Point3D>();

            Random random = new Random();
            double maximum = 5;
            double minimum = -5;

            for (int i = 0; i < 100; i++) {
                points.Add(new Point3D(random.NextDouble() * (maximum - minimum) + minimum, random.NextDouble() * (maximum - minimum) + minimum, random.NextDouble() * (maximum - minimum) + minimum));
            }

            Point3DCollection dataList = new Point3DCollection();
            PointsVisual3D cloudPoints = new PointsVisual3D { Color = Colors.White, Size = 3.0f };
            foreach (Point3D p in points) {
                dataList.Add(p);
            }
            cloudPoints.Points = dataList;

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
            string sensor_ip = textBoxSensorIP.Text;
            wrapper.GocatorManager_SetParameter(sensor_ip, 1);
            wrapper.GocatorManager_Init();
        }

        private void buttonRefresh_Click(object sender, RoutedEventArgs e) {
            string exposure = textBoxExposure.Text;
            wrapper.GocatorManager_SetParameter(exposure, 2);
        }

        private void buttonLoadPCL_Click(object sender, RoutedEventArgs e) {
            OpenFileDialog openFileDialog = new OpenFileDialog();
            if (openFileDialog.ShowDialog() == true) {
                strfilename = openFileDialog.FileName;
                wrapper.GocatorManager_LoadPointCloud(strfilename);
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
            checkInit();
            wrapper.GocatorManager_StartAcquisition(type);
        }

        private void buttonStopAcquisition_Click(object sender, RoutedEventArgs e) {
            wrapper.GocatorManager_StopAcquisition();
        }

        private void buttonFileAnalysis_Click(object sender, RoutedEventArgs e) {
            checkInit();
            wrapper.GocatorManager_FileAnalysis(type);
        }

        private void checkInit() {

            if (radioType1.IsChecked == true) {
                type = 1;
            } else {
                type = 2;
            }

            if (checkSaveFilePCL.IsChecked == true) {
                checkSavePCL = true;
            } else {
                checkSavePCL = false;
            }
        }
    }
}
