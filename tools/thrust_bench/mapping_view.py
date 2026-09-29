"""Manual channel identification controls; selections never start a motor."""
import tkinter as tk
from tkinter import ttk


def build_mapping_view(frame):
    box=ttk.LabelFrame(getattr(frame,"setup_area",frame),text="逐通道试转与上下桨映射",padding=6)
    box.pack(fill="x",pady=6)
    frame.raw_channel_text=tk.StringVar(frame,value="通道1：-- eRPM    通道2：-- eRPM")
    frame.mapping_status=tk.StringVar(frame,value="先逐路测试并观察，再选择对应桨叶和旋向。旋向只记录观察结果，不改变电机转向。")
    frame.mapping_percent=tk.StringVar(frame,value="5")
    frame.mapping_duration=tk.StringVar(frame,value="1.5")
    frame.mapping_confirmed=tk.BooleanVar(frame,value=False)
    frame.mapping_roles={ch:tk.StringVar(frame,value="请选择") for ch in (1,2)}
    frame.mapping_spins={ch:tk.StringVar(frame,value="请选择") for ch in (1,2)}
    frame._mapping_read_draft=None
    ttk.Label(box,textvariable=frame.raw_channel_text,wraplength=880).grid(row=0,column=0,columnspan=6,sticky="w")
    for column,text in enumerate(("电调通道","短时测试","观察到的桨叶","俯视旋向（从上往下看）")):
        ttk.Label(box,text=text).grid(row=1,column=column,sticky="w",padx=4,pady=3)
    for channel in (1,2):
        row=channel+1
        ttk.Label(box,text=f"通道{channel}").grid(row=row,column=0,padx=4,sticky="w")
        ttk.Button(box,text=f"测试通道{channel}",command=lambda ch=channel:frame.mapping_test(ch)).grid(row=row,column=1,padx=4,pady=2)
        ttk.Combobox(box,textvariable=frame.mapping_roles[channel],values=("请选择","上桨","下桨"),state="readonly",width=9).grid(row=row,column=2,padx=4)
        ttk.Combobox(box,textvariable=frame.mapping_spins[channel],values=("请选择","顺时针","逆时针"),state="readonly",width=12).grid(row=row,column=3,padx=4)
    options=ttk.Frame(box);options.grid(row=4,column=0,columnspan=6,sticky="ew",pady=4)
    ttk.Label(options,text="测试油门%（1～20）").pack(side="left")
    ttk.Entry(options,textvariable=frame.mapping_percent,width=5).pack(side="left",padx=4)
    ttk.Label(options,text="持续秒数（最多3秒）").pack(side="left",padx=(8,0))
    ttk.Entry(options,textvariable=frame.mapping_duration,width=5).pack(side="left",padx=4)
    ttk.Checkbutton(box,text="已固定台架并清场，确认本次低油门测试",variable=frame.mapping_confirmed).grid(row=5,column=0,columnspan=6,sticky="w")
    actions=ttk.Frame(box);actions.grid(row=6,column=0,columnspan=6,sticky="w",pady=4)
    for label,callback in (("读取当前映射",frame.mapping_read),("提交映射到RAM",frame.mapping_apply),("确认后保存到Flash",frame.mapping_save)):
        ttk.Button(actions,text=label,command=callback).pack(side="left",padx=(0,5))
    ttk.Label(box,textvariable=frame.mapping_status,wraplength=880).grid(row=7,column=0,columnspan=6,sticky="w")
