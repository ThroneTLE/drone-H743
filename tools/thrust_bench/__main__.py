from __future__ import annotations
import argparse, json
from pathlib import Path
from .session import read_samples
def parser():
    result=argparse.ArgumentParser(description="共轴推力台采集与离线分析"); sub=result.add_subparsers(dest="command")
    offline=sub.add_parser("analyze",help="从实际 samples.csv 生成离线模型报告"); offline.add_argument("samples",type=Path); offline.add_argument("--metadata",type=Path); offline.add_argument("--output",type=Path,required=True)
    return result
def main(argv=None):
    args=parser().parse_args(argv)
    if args.command=="analyze":
        from .model_report import write_analysis
        metadata=json.loads(args.metadata.read_text(encoding="utf-8")) if args.metadata else {}; outputs=write_analysis(read_samples(args.samples),metadata,args.output)
        print(json.dumps({k:str(v) for k,v in outputs.items()},ensure_ascii=False,indent=2)); return 0
    from tools.pressure_rs485_gui import main as ui_main
    ui_main(); return 0
if __name__=="__main__": raise SystemExit(main())
