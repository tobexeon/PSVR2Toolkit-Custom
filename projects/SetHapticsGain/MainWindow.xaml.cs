using System;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using PSVR2Toolkit.CAPI;

namespace SetHapticsGain
{
    public partial class MainWindow : Window
    {
        private bool _isUpdating = false;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            // 1. 加载图标 (现在统一调用 icon.ico)
            try
            {
                // 使用 pack URI 引用资源中的 icon.ico
                Uri iconUri = new Uri("pack://application:,,,/icon.ico", UriKind.RelativeOrAbsolute);
                this.Icon = BitmapFrame.Create(iconUri);
            }
            catch (Exception)
            {
                // 即使图标加载失败也不崩溃
            }

            // 2. 初始化界面数值和连接
            try
            {
                InputBox.Text = "1.00";
                SliderValueText.Text = "1.00";

                bool connected = IpcClient.Instance().Start();
                UpdateStatus(connected);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Startup Error: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void GainSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdating) return;
            _isUpdating = true;

            try 
            {
                double logVal = e.NewValue; 
                double actualGain = Math.Pow(10, logVal);

                if (InputBox != null)
                    InputBox.Text = actualGain.ToString("F2", CultureInfo.InvariantCulture);
                
                if (SliderValueText != null)
                    SliderValueText.Text = actualGain.ToString("F2", CultureInfo.InvariantCulture);
            }
            finally
            {
                _isUpdating = false;
            }
        }

        private void InputBox_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (_isUpdating || GainSlider == null) return;

            if (float.TryParse(InputBox.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out float val))
            {
                if (val < 0.1f) val = 0.1f;
                if (val > 10.0f) val = 10.0f;

                _isUpdating = true;
                
                double sliderPos = Math.Log10(val);
                
                if (sliderPos < -1) sliderPos = -1;
                if (sliderPos > 1) sliderPos = 1;

                GainSlider.Value = sliderPos;
                
                if (SliderValueText != null)
                    SliderValueText.Text = val.ToString("F2", CultureInfo.InvariantCulture);

                _isUpdating = false;
            }
        }

        private void ResetBtn_Click(object sender, RoutedEventArgs e)
        {
            _isUpdating = true;
            GainSlider.Value = 0;
            InputBox.Text = "1.00";
            SliderValueText.Text = "1.00";
            _isUpdating = false;
            
            SendGain(1.0f);
        }

        private void ApplyBtn_Click(object sender, RoutedEventArgs e)
        {
            if (float.TryParse(InputBox.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out float gain))
            {
                if (gain < 0.1f) gain = 0.1f;
                if (gain > 10.0f) gain = 10.0f;
                
                SendGain(gain);
            }
            else
            {
                MessageBox.Show("Invalid number format.", "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
            }
        }

        private void SendGain(float gain)
        {
            try
            {
                IpcClient.Instance().Start();
                IpcClient.Instance().SetHapticsGain(gain);
                UpdateStatus(true, $"Applied Gain: {gain:F2}");
            }
            catch (Exception ex)
            {
                UpdateStatus(false, $"Send Error: {ex.Message}");
            }
        }

        private void UpdateStatus(bool connected, string message = null)
        {
            if (StatusText == null) return;

            if (!connected)
            {
                StatusText.Text = "Disconnected (SteamVR not running?)";
                StatusText.Foreground = Brushes.Red;
            }
            else
            {
                StatusText.Text = message ?? "Connected & Ready";
                StatusText.Foreground = Brushes.Green;
            }
        }

        protected override void OnClosed(EventArgs e)
        {
            try { IpcClient.Instance().Stop(); } catch { }
            base.OnClosed(e);
        }
    }
}
