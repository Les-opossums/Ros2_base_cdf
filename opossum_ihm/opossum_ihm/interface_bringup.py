import tkinter as tk
import subprocess

class LaunchSelectorApp:
    def __init__(self, name="Launcher"):
        self.name = name
        self.root = tk.Tk()
        self.root.title(f"[{self.name}] Choix du plateau")

        # Gestion de l'écran (Plein écran si 480x800)
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        if screen_width == 480 and screen_height == 800:
            self.root.attributes("-fullscreen", True)
        else:
            self.root.geometry("480x800")
            
        self.label = tk.Label(self.root, text="Choisissez la configuration à lancer :", font=("Arial", 14))
        self.label.pack(pady=30)

        # Bouton Petit Plateau
        self.button_petit = tk.Button(
            self.root,
            text="Petit Plateau",
            bg="lightblue",
            font=("Arial", 16, "bold"),
            command=self.launch_petit_plateau,
            width=30,
            height=10,
        )
        self.button_petit.pack(pady=20)

        # Bouton Grand Plateau
        self.button_grand = tk.Button(
            self.root,
            text="Grand Plateau",
            bg="lightgreen",
            font=("Arial", 16, "bold"),
            command=self.launch_grand_plateau,
            width=30,
            height=10,
        )
        self.button_grand.pack(pady=20)

        self.root.mainloop()

    def launch_petit_plateau(self):
        commande = ["ros2", "launch", "opossum_bringup", "bringup_small_area.launch.py"]
        self.root.withdraw()
        process = subprocess.Popen(commande)
        process.wait() 
        self.root.destroy()

    def launch_grand_plateau(self):
        commande = ["ros2", "launch", "opossum_bringup", "bringup_simu.launch.py"]     
        self.root.withdraw()
        process = subprocess.Popen(commande)
        process.wait()
        self.root.destroy()

if __name__ == "__main__":
    app = LaunchSelectorApp("Robot Config")