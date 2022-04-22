using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Microsoft.Win32;
using Ookii.Dialogs.Wpf;

namespace GocatorGUI {

    struct Model {
        public int id;
        public List<PointsVisual3D> clouds;
        public List<string> row;

        public Model(int id) {
            this.id = id;
            clouds = new List<PointsVisual3D>();
            row = new List<string>();
        }
    }

    public partial class MainWindow : Window {

        private CWrapper wrapper = new CWrapper();
        private Pipe pipe = new Pipe();
        private Thread threadPipe;

        private int STRING_MAX_LENGTH = 1000;
        private int type = 11;
        private bool checkSavePCD = false;
        private string folderPathSavePCD = "";
        private bool checkWindowStateClick = false;

        private List<Model> listModel = new List<Model>();
        private bool first_model = true;
        private int pos = 0;
        private int index = 0;


        public MainWindow() {
            this.InitializeComponent();
            
            threadPipe = new Thread(delegate () {
                pipe.PipeRead(this);
            });
            threadPipe.Start();
        }

        public void AddPCL(List<Point3D> points, int id, int n) {

            if (n == 1) {
                Point3DCollection dataList = new Point3DCollection();
                PointsVisual3D cloudPoints = new PointsVisual3D { Color = Colors.White, Size = 1.0f };

                foreach (Point3D p in points) {
                    dataList.Add(p);
                }
                cloudPoints.Points = dataList;

                bool new_model = true;
                foreach (Model model in listModel) {
                    if (model.id == id) {
                        Console.WriteLine("Aggiungo ad una model esistente");
                        model.clouds.Add(cloudPoints);
                        new_model = false;
                        UpdateComboBox();
                    }
                }

                if (new_model & !first_model) {
                    Console.WriteLine("Inserisco una nuova model");
                    UpdateGraph(cloudPoints, id);
                    UpdateComboBox();
                    textBoxOutput.Text = "";
                    new_model = false;
                }

                if (first_model) {
                    Console.WriteLine("Inserisco la prima model");
                    UpdateGraph(cloudPoints, id);
                    UpdateComboBox();
                    first_model = false;
                }

            } else {

                Point3DCollection dataListMin = new Point3DCollection();
                PointsVisual3D cloudPointsMin = new PointsVisual3D { Color = Colors.Yellow, Size = 10.0f };
                dataListMin.Add(points[0]);
                cloudPointsMin.Points = dataListMin;

                Point3DCollection dataListMax = new Point3DCollection();
                PointsVisual3D cloudPointsMax = new PointsVisual3D { Color = Colors.Red, Size = 10.0f };
                dataListMax.Add(points[1]);
                cloudPointsMax.Points = dataListMax;

                foreach (Model model in listModel) {
                    if (model.id == id) {
                        Console.WriteLine("Inserisco min e max");
                        model.clouds.Add(cloudPointsMin);
                        model.clouds.Add(cloudPointsMax);
                        UpdateComboBox();
                    }
                }
            }
        }

        public void UpdateGraph(PointsVisual3D cloudPoints, int id) {
            var model = new Model(id);
            model.clouds.Add(cloudPoints);
            listModel.Add(model);

            // Add geometry to helixPlot. It renders asynchronously in the WPF composite render thread...
            ClearHelixViewport3D();
            helixPlot.Children.Add(cloudPoints);
            pos = listModel.Count - 1;
        }

        public void AddRow(string sent, int id) {

            listModel[listModel.Count - 1].row.Add(sent);

            textBoxOutput.AppendText(sent);
            textBoxOutput.AppendText(Environment.NewLine);
        }

        private void UpdateComboBox() {

            List<string> data = new List<string>();

            for (int i = 0; i < listModel[pos].clouds.Count; i++) {
                data.Add(i.ToString());
            }

            comboBox.SelectedIndex = 0;
            comboBox.ItemsSource = data;
        }

        private void ClearHelixViewport3D(){

            var converter = new System.Windows.Media.BrushConverter();

            helixPlot.Children.Clear();
            helixPlot.Children.Add(new DefaultLights());
            helixPlot.Children.Add(new GridLinesVisual3D { Width = 300, Length = 300, MajorDistance = 10, MinorDistance = 5, Thickness = 0.1, Fill = (Brush) converter.ConvertFromString("#FF696969") });
        }

        private void CheckType() {

            if (radioType1.IsChecked == true) {
                type = 11;
            } else if (radioType2.IsChecked == true) {
                type = 12;
            } else {
                type = 2;
            }
        }

        // Panel Window
        private void Border_MouseDown(object sender, MouseButtonEventArgs e) {
            if (e.LeftButton == MouseButtonState.Pressed) {
                DragMove();
            }
        }

        private void ButtonMinimize_Click(object sender, RoutedEventArgs e) {
            Application.Current.MainWindow.WindowState = WindowState.Minimized;
        }

        private void ButtonWindowState_Click(object sender, RoutedEventArgs e) {
            
            if (!checkWindowStateClick) {
                double x = SystemParameters.WorkArea.Width;
                double y = SystemParameters.WorkArea.Height;

                Application.Current.MainWindow.WindowState = WindowState.Normal;
                Application.Current.MainWindow.Top = 0;
                Application.Current.MainWindow.Left = 0;
                Application.Current.MainWindow.Width = x;
                Application.Current.MainWindow.Height = y;

                checkWindowStateClick = true;
            } else {

                Application.Current.MainWindow.WindowState = WindowState.Normal;
                Application.Current.MainWindow.Top = 0;
                Application.Current.MainWindow.Left = 0;
                Application.Current.MainWindow.Width = 1280;
                Application.Current.MainWindow.Height = 630;

                checkWindowStateClick = false;
            }
        }

        private void ButtonClose_Click(object sender, RoutedEventArgs e) {
            pipe.thread_saving_active = false;
            //pipe.threadUpdate.Join();
            //threadPipe.Join();
            Application.Current.Shutdown();
            Environment.Exit(0);
        }

        // Panel Config
        private void ButtonConnect_Click(object sender, RoutedEventArgs e) {
            
            StringBuilder message = new StringBuilder(STRING_MAX_LENGTH);
            string sensor_ip = textBoxSensorIP.Text;

            wrapper.GocatorManager_SetParameter(message, STRING_MAX_LENGTH, sensor_ip, 1);
            wrapper.GocatorManager_Init(message, STRING_MAX_LENGTH);

            if (message.ToString().Equals("OK")) {
                textBoxExposure.IsEnabled = true;
                buttonRefresh.IsEnabled = true;
                buttonStartAcquisition.IsEnabled = true;
                buttonStopAcquisition.IsEnabled = true;
            } else {
                labelConnect.Foreground = new SolidColorBrush(Colors.Red);
            }
            labelConnect.Content = message.ToString().Replace('-', '\n');
            labelConnect.Visibility = Visibility.Visible;
        }

        private void ButtonRefresh_Click(object sender, RoutedEventArgs e) {

            StringBuilder message = new StringBuilder(STRING_MAX_LENGTH);
            string exposure = textBoxExposure.Text;

            wrapper.GocatorManager_SetParameter(message, STRING_MAX_LENGTH, exposure, 2);

            if (!message.ToString().Equals("OK")) {
                labelRefresh.Foreground = new SolidColorBrush(Colors.Red);
            }
            labelRefresh.Content = message.ToString().Replace('-', '\n');
            labelRefresh.Visibility = Visibility.Visible;
        }

        private void ButtonLoadPCD_Click(object sender, RoutedEventArgs e) {

            StringBuilder message = new StringBuilder(STRING_MAX_LENGTH);
            OpenFileDialog openFileDialog = new OpenFileDialog();
            
            if (openFileDialog.ShowDialog() == true) {

                Mouse.OverrideCursor = Cursors.Wait;
                string strfilename = openFileDialog.FileName;
                wrapper.GocatorManager_LoadPointCloud(message, STRING_MAX_LENGTH, strfilename);
                
                if (message.ToString().Equals("OK")) {
                    buttonFileAnalysis.IsEnabled = true;
                } else {
                    labelLoadPCL.Foreground = new SolidColorBrush(Colors.Red);
                }
                labelLoadPCL.Content = message.ToString().Replace('-', '\n');
                labelLoadPCL.Visibility = Visibility.Visible;
            }

            Mouse.OverrideCursor = Cursors.Arrow;
        }

        private void CheckSaveFilePCD_Click(object sender, RoutedEventArgs e) {

            if (!checkSavePCD) {
                checkSavePCD = true;
                buttonSelectFolder.IsEnabled = true;
            } else {
                checkSavePCD = false;
                buttonSelectFolder.IsEnabled = false;
            }
        }

        private void ButtonSelectFolder_Click(object sender, RoutedEventArgs e) {

            var folderBrowser = new VistaFolderBrowserDialog();

            bool? success = folderBrowser.ShowDialog();

            if (success == true) {

                folderPathSavePCD = folderBrowser.SelectedPath;
                Console.WriteLine(folderBrowser.SelectedPath);
            }
        }

        // Button on Bottom
        private void ComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e) {
            
            var selectedcomboitem = sender as ComboBox;
            index = selectedcomboitem.SelectedIndex;

            ClearHelixViewport3D();

            switch (index) {
                
                case 1:
                    helixPlot.Children.Add(listModel[pos].clouds[0]);
                    helixPlot.Children.Add(listModel[pos].clouds[1]);
                    helixPlot.Children.Add(listModel[pos].clouds[2]);
                    break;

                case 2:
                    helixPlot.Children.Add(listModel[pos].clouds[1]);
                    helixPlot.Children.Add(listModel[pos].clouds[2]);
                    break;

                default:
                    helixPlot.Children.Add(listModel[pos].clouds[index]);
                    break;
            }
        }

        private void ButtonBackward_Click(object sender, RoutedEventArgs e) {

            if (pos != 0) {
                ClearHelixViewport3D();
                helixPlot.Children.Add(listModel[pos - 1].clouds[0]);

                textBoxOutput.Text = "";
                for (int i = 0; i < listModel[pos - 1].row.Count; i++) {
                    textBoxOutput.AppendText(listModel[pos - 1].row[i]);
                    textBoxOutput.AppendText(Environment.NewLine);
                }

                pos--;
                UpdateComboBox();
            }

            Console.WriteLine(pos);
        }

        private void ButtonForward_Click(object sender, RoutedEventArgs e) {

            if (pos < listModel.Count - 1) {
                ClearHelixViewport3D();
                helixPlot.Children.Add(listModel[pos + 1].clouds[0]);

                textBoxOutput.Text = "";
                for (int i = 0; i < listModel[pos + 1].row.Count; i++) {
                    textBoxOutput.AppendText(listModel[pos + 1].row[i]);
                    textBoxOutput.AppendText(Environment.NewLine);
                }

                pos++;
                UpdateComboBox();
            }

            Console.WriteLine(pos);
        }
        
        private void ButtonReset_Click(object sender, RoutedEventArgs e) {

            ClearHelixViewport3D();
            textBoxOutput.Text = "";
            listModel.Clear();
            first_model = true;
            pos = 0;
            index = 0;
        }

        private void ButtonStartAcquisition_Click(object sender, RoutedEventArgs e) {

            StringBuilder message = new StringBuilder(STRING_MAX_LENGTH);

            CheckType();
            wrapper.GocatorManager_StartAcquisition(message, STRING_MAX_LENGTH, type, checkSavePCD, folderPathSavePCD);

            if (!message.ToString().Equals("OK")) {
                MessageBox.Show(message.ToString());
            }
        }

        private void ButtonStopAcquisition_Click(object sender, RoutedEventArgs e) {

            StringBuilder message = new StringBuilder(STRING_MAX_LENGTH);

            wrapper.GocatorManager_StopAcquisition(message, STRING_MAX_LENGTH);

            if (!message.ToString().Equals("OK")) {
                MessageBox.Show(message.ToString());
            }
        }

        private void ButtonFileAnalysis_Click(object sender, RoutedEventArgs e) {

            CheckType();
            Mouse.OverrideCursor = Cursors.Wait;
            wrapper.GocatorManager_FileAnalysis(type, checkSavePCD, folderPathSavePCD);
            Mouse.OverrideCursor = Cursors.Arrow;
        }
    }
}
