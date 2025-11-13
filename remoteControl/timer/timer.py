import tkinter as tk
from tkinter import simpledialog, messagebox
import time
from datetime import datetime, date
import winsound
import json
import os

class TimerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Task Timer")
        self.root.geometry("600x500")  # Increased height to accommodate daily objective
        
        # VS Code Dark Theme Colors
        self.bg_color = "#1e1e1e"  # Main background
        self.sidebar_bg = "#252526"  # Sidebar background
        self.text_bg = "#1e1e1e"  # Text area background
        self.text_fg = "#d4d4d4"  # Text foreground
        self.accent_color = "#007acc"  # VS Code blue
        self.button_bg = "#0e639c"  # Button background
        self.button_hover = "#1177bb"  # Button hover
        self.label_fg = "#cccccc"  # Label text color
        self.border_color = "#3c3c3c"  # Border color
        self.success_color = "#4EC9B0"  # Success green
        self.warning_color = "#CE9178"  # Warning orange
        
        # Configure root background
        self.root.configure(bg=self.bg_color)
        
        # Daily objective section
        self.objective_frame = tk.Frame(root, relief=tk.RAISED, bd=1, 
                                       bg=self.sidebar_bg, highlightbackground=self.border_color,
                                       highlightthickness=1)
        self.objective_frame.pack(pady=10, fill=tk.X, padx=20)
        
        self.objective_label = tk.Label(self.objective_frame, text="Daily Objective:", 
                                       font=("Segoe UI", 12, "bold"), bg=self.sidebar_bg, 
                                       fg=self.label_fg)
        self.objective_label.pack(anchor=tk.W, padx=5, pady=(5,0))
        
        self.objective_text = tk.Label(self.objective_frame, text="No objective set", 
                                      wraplength=500, font=("Segoe UI", 10), 
                                      fg=self.accent_color, bg=self.sidebar_bg, 
                                      height=2, justify=tk.LEFT)
        self.objective_text.pack(fill=tk.X, padx=5)
        
        self.complete_button = tk.Button(self.objective_frame, text="Complete Objective", 
                                        command=self.complete_objective, state=tk.DISABLED,
                                        bg=self.button_bg, fg="white", font=("Segoe UI", 9),
                                        relief=tk.FLAT, padx=10, pady=5)
        self.complete_button.pack(pady=5)
        
        # Task section
        self.task_label = tk.Label(root, text="Current Task:", bg=self.bg_color, 
                                  fg=self.label_fg, font=("Segoe UI", 11))
        self.task_label.pack(pady=(10,5))
        
        self.task_text = tk.Text(root, height=4, width=50, bg=self.text_bg, fg=self.text_fg,
                                font=("Consolas", 10), insertbackground=self.text_fg,
                                relief=tk.FLAT, highlightbackground=self.border_color,
                                highlightthickness=1, padx=5, pady=5)
        self.task_text.pack()
        
        self.interval_label = tk.Label(root, text="Time interval: ", bg=self.bg_color, 
                                      fg=self.label_fg, font=("Segoe UI", 11))
        self.interval_label.pack(pady=(15,5))
        
        self.intervals = ["5 minutes", "10 minutes", "15 minutes", "20 minutes", "30 minutes", "1 hour"]
        self.interval_var = tk.StringVar(root)
        self.interval_var.set(self.intervals[0])  # Default value
        
        # Custom style for dropdown
        self.root.option_add('*TCombobox*Listbox.background', self.text_bg)
        self.root.option_add('*TCombobox*Listbox.foreground', self.text_fg)
        self.root.option_add('*TCombobox*Listbox.selectBackground', self.accent_color)
        self.root.option_add('*TCombobox*Listbox.selectForeground', 'white')
        
        self.interval_dropdown = tk.OptionMenu(root, self.interval_var, *self.intervals)
        self.interval_dropdown.config(bg=self.button_bg, fg="white", font=("Segoe UI", 9),
                                     relief=tk.FLAT, highlightbackground=self.border_color,
                                     highlightthickness=1)
        self.interval_dropdown.pack(pady=5)
        
        self.start_button = tk.Button(root, text="Start Timer", command=self.start_timer,
                                     bg=self.success_color, fg="white", font=("Segoe UI", 10, "bold"),
                                     relief=tk.FLAT, padx=20, pady=8)
        self.start_button.pack(pady=15)
        
        self.countdown_label = tk.Label(root, text="", font=("Segoe UI", 48, "bold"),
                                       bg=self.bg_color, fg=self.accent_color)
        self.countdown_label.pack(pady=20)
        
        # Add hover effects to buttons
        self.setup_hover_effects()
        
        self.log_file = "task_log.txt"
        self.objective_file = "daily_objectives.json"
        self.start_time = None
        self.remaining_time = 0
        self.current_task = None
        self.idle_check_time = 300  # 5 minutes idle check interval
        self.timer_running = False
        self.after_id = None
        
        # Check and set daily objective
        self.check_daily_objective()

    def setup_hover_effects(self):
        # Configure button hover effects
        buttons = [self.complete_button, self.start_button, self.interval_dropdown]
        
        for button in buttons:
            # Bind events for hover effect
            button.bind("<Enter>", lambda e, b=button: self.on_enter(e, b))
            button.bind("<Leave>", lambda e, b=button: self.on_leave(e, b))
    
    def on_enter(self, event, button):
        """Change button color on hover"""
        if button == self.start_button:
            button.config(bg="#5FD7B0")  # Lighter green
        elif button == self.complete_button and button['state'] == tk.NORMAL:
            button.config(bg=self.button_hover)
        elif button == self.interval_dropdown:
            button.config(bg="#1f7bbb")
    
    def on_leave(self, event, button):
        """Revert button color when not hovering"""
        if button == self.start_button:
            button.config(bg=self.success_color)
        elif button == self.complete_button:
            if button['state'] == tk.NORMAL:
                button.config(bg=self.button_bg)
        elif button == self.interval_dropdown:
            button.config(bg=self.button_bg)

    def check_daily_objective(self):
        """检查是否需要设置每日目标"""
        today = date.today().isoformat()
        
        # 加载之前的目标数据
        objectives = self.load_objectives()
        
        if today in objectives:
            # 今天已经有目标了，显示它
            self.current_objective = objectives[today]['objective']
            self.objective_completed = objectives[today]['completed']
            self.display_objective()
        else:
            # 今天还没有目标，询问用户
            self.set_daily_objective()

    def load_objectives(self):
        """从文件加载目标数据"""
        if os.path.exists(self.objective_file):
            try:
                with open(self.objective_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except:
                return {}
        return {}

    def save_objectives(self, objectives):
        """保存目标数据到文件"""
        with open(self.objective_file, 'w', encoding='utf-8') as f:
            json.dump(objectives, f, ensure_ascii=False, indent=2)

    def set_daily_objective(self):
        """设置每日目标"""
        # Create a custom dialog with dark theme
        dialog = tk.Toplevel(self.root)
        dialog.title("Daily Objective")
        dialog.configure(bg=self.bg_color)
        dialog.geometry("400x200")
        dialog.transient(self.root)
        dialog.grab_set()
        
        # Center the dialog
        dialog.geometry("+%d+%d" % (self.root.winfo_rootx() + 100, 
                                   self.root.winfo_rooty() + 100))
        
        label = tk.Label(dialog, text="What's your objective for today?", 
                        bg=self.bg_color, fg=self.label_fg, font=("Segoe UI", 11))
        label.pack(pady=20)
        
        entry = tk.Entry(dialog, width=40, bg=self.text_bg, fg=self.text_fg,
                        font=("Segoe UI", 10), relief=tk.FLAT, 
                        highlightbackground=self.border_color, highlightthickness=1)
        entry.pack(pady=10, padx=20, fill=tk.X)
        entry.focus_set()
        
        def on_ok():
            objective = entry.get().strip()
            if objective:
                self.current_objective = objective
                self.objective_completed = False
                
                # 保存到文件
                objectives = self.load_objectives()
                today = date.today().isoformat()
                objectives[today] = {
                    'objective': objective,
                    'completed': False,
                    'date': today
                }
                self.save_objectives(objectives)
                
                self.display_objective()
                dialog.destroy()
            else:
                messagebox.showwarning("Empty Objective", "Please enter an objective.", parent=dialog)
        
        def on_cancel():
            # If user cancels, ask again later
            dialog.destroy()
            self.root.after(1000, self.set_daily_objective)
        
        button_frame = tk.Frame(dialog, bg=self.bg_color)
        button_frame.pack(pady=20)
        
        ok_btn = tk.Button(button_frame, text="OK", command=on_ok, bg=self.button_bg, 
                          fg="white", font=("Segoe UI", 9), relief=tk.FLAT, padx=15)
        ok_btn.pack(side=tk.LEFT, padx=10)
        
        cancel_btn = tk.Button(button_frame, text="Cancel", command=on_cancel, 
                              bg=self.sidebar_bg, fg=self.label_fg, font=("Segoe UI", 9), 
                              relief=tk.FLAT, padx=15)
        cancel_btn.pack(side=tk.LEFT, padx=10)
        
        dialog.bind('<Return>', lambda e: on_ok())
        dialog.bind('<Escape>', lambda e: on_cancel())

    def display_objective(self):
        """显示当前目标"""
        if hasattr(self, 'current_objective'):
            status = "✓ Completed" if self.objective_completed else "○ In Progress"
            color = self.success_color if self.objective_completed else self.accent_color
            self.objective_text.config(text=f"{self.current_objective} - {status}", fg=color)
            
            # 启用或禁用完成按钮
            if self.objective_completed:
                self.complete_button.config(state=tk.DISABLED, text="Objective Completed",
                                          bg=self.sidebar_bg, fg=self.border_color)
            else:
                self.complete_button.config(state=tk.NORMAL, text="Complete Objective",
                                          bg=self.button_bg, fg="white")

    def complete_objective(self):
        """标记目标为完成"""
        if hasattr(self, 'current_objective') and not self.objective_completed:
            self.objective_completed = True
            
            # 更新文件
            objectives = self.load_objectives()
            today = date.today().isoformat()
            if today in objectives:
                objectives[today]['completed'] = True
                self.save_objectives(objectives)
            
            # 记录到日志
            self.log_task(f"Daily Objective: {self.current_objective}", 0, is_objective=True)
            
            # 更新显示
            self.display_objective()
            
            messagebox.showinfo("Objective Completed", "Congratulations! You've completed your daily objective!")

    def get_task(self):
        task = self.task_text.get("1.0", tk.END).strip()
        interval_str = self.interval_var.get()
        interval_map = {
            "5 minutes": 300,
            "10 minutes": 600,
            "15 minutes": 900,
            "20 minutes": 1200,
            "30 minutes": 1800,
            "1 hour": 3600
        }
        interval = interval_map[interval_str]
        return task, interval

    def log_task(self, task, duration, is_objective=False):
        # 修复中文乱码问题，使用utf-8编码
        with open(self.log_file, "a", encoding="utf-8") as log_file:
            if is_objective:
                log_file.write(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - 🎯 完成目标: {task}\n")
            else:
                # 将秒数转换为更易读的格式
                hours = duration // 3600
                minutes = (duration % 3600) // 60
                seconds = duration % 60
                
                if hours > 0:
                    duration_str = f"{hours}小时{minutes}分钟{seconds}秒"
                elif minutes > 0:
                    duration_str = f"{minutes}分钟{seconds}秒"
                else:
                    duration_str = f"{seconds}秒"
                    
                log_file.write(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - 任务: {task}, 时长: {duration_str}\n")

    def start_timer(self):
        if self.timer_running:
            messagebox.showinfo("Timer Running", "计时器已经在运行中！")
            return
            
        task, interval = self.get_task()
        if task:
            self.start_time = time.time()
            self.remaining_time = interval
            self.current_task = task
            self.timer_running = True
            self.start_button.config(state=tk.DISABLED, bg=self.border_color, text="Timer Running...")
            self.update_countdown()
        else:
            messagebox.showwarning("No Task", "Please enter a task to start the timer.")

        self.root.after(self.idle_check_time * 1000, self.check_idle)

    def update_countdown(self):
        if self.timer_running and self.remaining_time > 0:
            mins, secs = divmod(self.remaining_time, 60)
            time_format = '{:02d}:{:02d}'.format(mins, secs)
            self.countdown_label.config(text=time_format)
            
            # Change color when under 1 minute
            if self.remaining_time <= 60:
                self.countdown_label.config(fg=self.warning_color)
            else:
                self.countdown_label.config(fg=self.accent_color)
                
            self.remaining_time -= 1
            self.after_id = self.root.after(1000, self.update_countdown)
        elif self.remaining_time <= 0 and self.timer_running:
            self.countdown_label.config(text="Time's up!", fg=self.warning_color)
            self.timer_ended()

    def timer_ended(self):
        self.timer_running = False
        if self.after_id:
            self.root.after_cancel(self.after_id)
            self.after_id = None
            
        self.start_button.config(state=tk.NORMAL, bg=self.success_color, text="Start Timer")
        winsound.Beep(1000, 1000)  # Notification sound when the time is up
        response = messagebox.askyesno("Timer Ended", f"Did you finish the task '{self.current_task}'?")
        if response:
            # 任务完成，记录日志并询问下一个任务
            self.log_task(self.current_task, int(time.time() - self.start_time))
            self.ask_new_task()
        else:
            extend = messagebox.askyesno("Extend Timer", "Are you still working on it and want to add more time?")
            if extend:
                self.extend_timer()
            else:
                # 任务未完成但不再延长，记录日志并询问下一个任务
                self.log_task(self.current_task, int(time.time() - self.start_time))
                self.ask_new_task()

    def extend_timer(self):
        # 修复：使用更简单的方法获取额外时间
        extra_minutes = simpledialog.askinteger("Add Time", "Enter additional minutes:", minvalue=1, maxvalue=120)
        if extra_minutes:
            extra_time = extra_minutes * 60  # 转换为秒
            self.remaining_time += extra_time
            self.timer_running = True
            self.countdown_label.config(fg=self.accent_color)
            self.start_button.config(state=tk.DISABLED, bg=self.border_color, text="Timer Running...")
            self.update_countdown()
            messagebox.showinfo("Timer Extended", f"Added {extra_minutes} minutes to the timer.")

    def ask_new_task(self):
        # 任务完成后询问新任务
        self.task_text.delete("1.0", tk.END)
        self.countdown_label.config(text="", fg=self.accent_color)
        messagebox.showinfo("Next Task", "Please enter your next task and click Start.")

    def check_idle(self):
        if not self.timer_running:
            current_task = self.task_text.get("1.0", tk.END).strip()
            if not current_task:
                winsound.Beep(1000, 1000)
                messagebox.showwarning("Idle Warning", "No task set within the last 5 minutes!")
        self.root.after(self.idle_check_time * 1000, self.check_idle)

if __name__ == "__main__":
    root = tk.Tk()
    app = TimerApp(root)
    root.mainloop()