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
        
        # Daily objective section
        self.objective_frame = tk.Frame(root, relief=tk.RAISED, bd=1)
        self.objective_frame.pack(pady=10, fill=tk.X, padx=20)
        
        self.objective_label = tk.Label(self.objective_frame, text="Daily Objective:", font=("Helvetica", 12, "bold"))
        self.objective_label.pack(anchor=tk.W)
        
        self.objective_text = tk.Label(self.objective_frame, text="No objective set", wraplength=500, 
                                      font=("Helvetica", 10), fg="blue", height=2)
        self.objective_text.pack(fill=tk.X, padx=5)
        
        self.complete_button = tk.Button(self.objective_frame, text="Complete Objective", 
                                        command=self.complete_objective, state=tk.DISABLED)
        self.complete_button.pack(pady=5)
        
        # Task section
        self.task_label = tk.Label(root, text="Current Task:")
        self.task_label.pack()
        
        self.task_text = tk.Text(root, height=4, width=50)
        self.task_text.pack()
        
        self.interval_label = tk.Label(root, text="Time interval: ")
        self.interval_label.pack()
        
        self.intervals = ["5 minutes", "10 minutes", "15 minutes", "20 minutes", "30 minutes", "1 hour"]
        self.interval_var = tk.StringVar(root)
        self.interval_var.set(self.intervals[0])  # Default value
        
        self.interval_dropdown = tk.OptionMenu(root, self.interval_var, *self.intervals)
        self.interval_dropdown.pack()
        
        self.start_button = tk.Button(root, text="Start", command=self.start_timer)
        self.start_button.pack()
        
        self.countdown_label = tk.Label(root, text="", font=("Helvetica", 48))
        self.countdown_label.pack(pady=20)
        
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
        objective = simpledialog.askstring("Daily Objective", 
                                          "What's your objective for today?",
                                          initialvalue="")
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
        else:
            # 如果用户没有输入，稍后再次询问
            self.root.after(1000, self.set_daily_objective)

    def display_objective(self):
        """显示当前目标"""
        if hasattr(self, 'current_objective'):
            status = "✓ Completed" if self.objective_completed else "○ In Progress"
            color = "green" if self.objective_completed else "blue"
            self.objective_text.config(text=f"{self.current_objective} - {status}", fg=color)
            
            # 启用或禁用完成按钮
            if self.objective_completed:
                self.complete_button.config(state=tk.DISABLED, text="Objective Completed")
            else:
                self.complete_button.config(state=tk.NORMAL, text="Complete Objective")

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
            self.update_countdown()
        else:
            messagebox.showwarning("No Task", "Please enter a task to start the timer.")

        self.root.after(self.idle_check_time * 1000, self.check_idle)

    def update_countdown(self):
        if self.timer_running and self.remaining_time > 0:
            mins, secs = divmod(self.remaining_time, 60)
            time_format = '{:02d}:{:02d}'.format(mins, secs)
            self.countdown_label.config(text=time_format)
            self.remaining_time -= 1
            self.after_id = self.root.after(1000, self.update_countdown)
        elif self.remaining_time <= 0 and self.timer_running:
            self.countdown_label.config(text="Time's up!")
            self.timer_ended()

    def timer_ended(self):
        self.timer_running = False
        if self.after_id:
            self.root.after_cancel(self.after_id)
            self.after_id = None
            
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
            self.update_countdown()
            messagebox.showinfo("Timer Extended", f"Added {extra_minutes} minutes to the timer.")

    def ask_new_task(self):
        # 任务完成后询问新任务
        self.task_text.delete("1.0", tk.END)
        self.countdown_label.config(text="")
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