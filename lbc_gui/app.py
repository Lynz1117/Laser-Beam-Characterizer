import argparse, sys
from PySide6.QtWidgets import QApplication
from .main_window import MainWindow

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--simulate", action="store_true", help="Run with simulator backend")
    p.add_argument("--port", help="Serial port (e.g., COM3)"); p.add_argument("--baud", type=int, default=115200)
    args = p.parse_args()

    app = QApplication(sys.argv)
    win = MainWindow(simulate=args.simulate, port=args.port, baud=args.baud)
    win.show()
    sys.exit(app.exec())

    if __name__ == "__main__":
        main()