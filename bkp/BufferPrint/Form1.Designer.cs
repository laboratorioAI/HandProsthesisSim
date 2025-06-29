namespace BufferPrint
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
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
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            label1 = new Label();
            label2 = new Label();
            button1 = new Button();
            button2 = new Button();

            SuspendLayout();
            // 
            // label1
            // 
            label1.AutoSize = true;
            label1.Location = new Point(31, 29);
            label1.Name = "label1";
            label1.Size = new Size(79, 20);
            label1.TabIndex = 0;
            label1.Text = "BufferPrint";
            //label1.Click += label1_Click;
            // 
            // label2
            // 
            label2.AutoSize = true;
            label2.Location = new Point(87, 76);
            label2.Name = "label2";
            label2.Size = new Size(173, 20);
            label2.TabIndex = 1;
            label2.Text = "Here Print Buffer content";
            // 
            // button1
            // 
            button1.Location = new Point(58, 164);
            button1.Name = "button1";
            button1.Size = new Size(94, 29);
            button1.TabIndex = 2;
            button1.Text = "Start";
            button1.UseVisualStyleBackColor = true;
            button1.Click += StartButton_Click;
            // 
            // button2
            // 
            button2.Location = new Point(235, 173);
            button2.Name = "button2";
            button2.Size = new Size(94, 29);
            button2.TabIndex = 3;
            button2.Text = "Stop";
            button2.UseVisualStyleBackColor = true;
            button2.Click += StopButton_Click;
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(8F, 20F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(337, 248);
            Controls.Add(button2);
            Controls.Add(button1);
            Controls.Add(label2);
            Controls.Add(label1);
            Name = "Form1";
            Text = "Form1";
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private Label label1;
        private Label label2;
        private Button button1;
        private Button button2;

        private bool isRunning = false;
        private BufferManager _bufferManager;

        public Form1(BufferManager bufferManager)
        {
            _bufferManager = bufferManager;
            InitializeComponent();
        }

        private async void StartButton_Click(object sender, EventArgs e)
        {
            // Start the continuous buffer popping loop
            isRunning = true; // Enable the loop
            await Task.Run(() => RunBufferLoop());
        }

        private void StopButton_Click(object sender, EventArgs e)
        {
            // Stop the loop by setting isRunning to false
            isRunning = false;
        }

        private async Task RunBufferLoop()
        {
            while (isRunning)
            {
                string poppedValue = _bufferManager.PopBuffer();

                // Update the label2 text on the UI thread
                Invoke((Action)(() =>
                {
                    label2.Text = poppedValue;
                }));

                // Simulate a small delay to prevent the loop from running too fast
                await Task.Delay(1000); // Adjust delay as needed
            }
        }

    }
}
