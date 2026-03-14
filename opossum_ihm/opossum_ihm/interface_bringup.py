import tkinter as tk
import subprocess
import os

class LaunchSelectorApp:
    def __init__(self, name="Launcher"):
        self.name = name
        self.root = tk.Tk()
        self.root.title(f"[{self.name}] Choix du plateau")

        # Fichier qui stockera l'état
        self.config_file = "/home/opossum/robot_ws/config_plateau.txt"

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
            command=lambda: self.launch_service("petit"),
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
            command=lambda: self.launch_service("grand"),
            width=30,
            height=10,
        )
        self.button_grand.pack(pady=20)

        self.root.mainloop()

    def launch_service(self, config_choice):
        # 1. Écrire le choix dans le fichier texte
        with open(self.config_file, "w") as f:
            f.write(config_choice)

        # 2. Redémarrer le user service systemd
        # Notez l'ajout de "--user" et le retrait de "sudo"
        commande = ["systemctl", "--user", "restart", "launch.service"]
        
        self.root.withdraw() # Cache la fenêtre
        subprocess.run(commande) # Lance la commande
        self.root.destroy() # Ferme l'application Tkinter

if __name__ == "__main__":
    app = LaunchSelectorApp("Robot Config")