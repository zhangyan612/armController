﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

namespace CSharp_API_Example
{
    public partial class ScreenShareView : UserControl
    {
        public ScreenShareView()
        {
            InitializeComponent();
        }

        public void SetUIText(ConfigHelper config)
        {
            this.label1.Text = config.GetUIValue("General", "ScreenShare.Title");
        }
    }
}
