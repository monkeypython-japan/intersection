import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from ui.app import App


def main():
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()
