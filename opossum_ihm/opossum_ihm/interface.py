import tkinter as tk
from PIL import Image, ImageTk
import os
from itertools import count, cycle
from ament_index_python.packages import get_package_share_directory
from rclpy.logging import get_logger


class ColorChoiceApp:
    def __init__(self):
        self.reload = False
        self.root = tk.Tk()
        self.root.title("Choisir une couleur")

        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        if screen_width == 480 and screen_height == 800:
            # Plein écran
            self.root.attributes("-fullscreen", True)
        else:
            self.root.geometry("480x800")
        self.label = tk.Label(self.root, text="Choisissez une couleur :")
        self.label.pack(pady=10)

        self.button_yellow = tk.Button(
            self.root,
            text="Yellow",
            bg="yellow",
            command=lambda: self.chs_clr("yellow"),
            width=40,
            height=15,
        )
        self.button_yellow.pack(padx=20)

        self.button_blue = tk.Button(
            self.root,
            text="Blue",
            bg="blue",
            command=lambda: self.chs_clr("blue"),
            width=40,
            height=15,
        )
        self.button_blue.pack(padx=20)

        self.selected_color = None

        self.button_reload = tk.Button(
            self.root, text="Reload", command=self.reload_ihm
        )
        self.button_reload.pack(pady=10)

        # Bouton pour quitter
        self.button_quit = tk.Button(self.root,
                                     text="Quit",
                                     command=self.root.destroy)
        self.button_quit.pack(pady=20)
        self.root.mainloop()

    def chs_clr(self, color):
        self.selected_color = color
        self.root.destroy()

    def reload_ihm(self):
        self.root.destroy()
        self.reload = True


class ImageApp:
    def __init__(self, selected_color):
        self.reload = False
        self.selected_script = 0
        self.selected_color = selected_color

        self.root = tk.Tk()
        self.root.title("Choose your script")
        self.root.configure(bg=selected_color)
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()

        if screen_width == 480 and screen_height == 800:
            # Plein écran
            self.root.attributes("-fullscreen", True)
        else:
            self.root.geometry("480x800")
        self.color = selected_color
        plateau_path = os.path.join(
            get_package_share_directory("opossum_ihm"),
            "images",
            "plateau.png",
        )
        self.img = Image.open(plateau_path)
        # Pivoter l'image de 90° dans le sens des aiguilles d'une montre
        self.img = self.img.rotate(90, expand=True)
        # Dimensions de la fenêtre
        window_width = 480
        window_height = 800
        # Dimensions de l'image après la rotation
        img_width, img_height = self.img.size
        # Calculer les facteurs de redimensionnement
        width_factor = window_width / img_width
        height_factor = window_height / img_height
        # Choisir le plus petit facteur pour conserver le ratio
        scale_factor = min(width_factor, height_factor) * 0.9
        # Redimensionner l'image en maintenant le ratio
        new_width = int(img_width * scale_factor)
        new_height = int(img_height * scale_factor)
        # Redimensionner l'image
        self.img = self.img.resize((new_width, new_height))
        # Convertir l'image en format compatible avec Tkinter
        self.tk_img = ImageTk.PhotoImage(self.img)
        # Créer un canevas pour afficher l'image
        self.canvas = tk.Canvas(self.root, width=new_width, height=new_height)
        self.canvas.pack()
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_img)
        # Bind pour obtenir les coordonnées du clic
        self.canvas.bind("<Button-1>", self.get_coordinates)
        # Label pour afficher les coordonnées
        self.label = tk.Label(self.root, text=f"Color: {self.color}")
        self.label.pack(pady=10)

        self.button_reload = tk.Button(
            self.root,
            text="Reload",
            command=self.reload_ihm,
            height=3,
            width=10
        )
        self.button_reload.pack(padx=0, pady=10)
        self.button_reload.place(x=100, y=725)

        # Bouton pour quitter
        self.button_valid = tk.Button(
            self.root,
            text="Valid",
            command=self.root.destroy,
            height=3,
            width=10
        )
        self.button_valid.pack(padx=20, pady=10)
        self.button_valid.place(x=300, y=725)
        self.root.mainloop()

    def get_coordinates(self, event):
        x, y = event.x, event.y
        self.label.config(text=f"Couleur: {self.color} | x={x}, y={y})")
        print(f"Couleur choisie: {self.color}, Coordonnées: ({x}, {y})")
        if x < 105 and y > 515 and y < 620:
            self.selected_script = 1
        elif x > 190 and x < 300 and y > 545:
            self.selected_script = 2
        elif x > 350 and y > 330 and y < 450:
            self.selected_script = 3
        elif x > 350 and y > 185 and y < 390:
            self.selected_script = 4
        elif x > 190 and x < 300 and y < 80:
            self.selected_script = 5
        elif x < 105 and y < 140 and y > 30:
            self.selected_script = 6
        else:
            self.selected_script = 0
        self.label.config(text=f"Script: {self.selected_script}")

    def reload_ihm(self):
        self.root.destroy()
        self.reload = True


class ValidationApp:
    def __init__(self, color, script):
        self.color = color
        self.script = script
        self.reload = False

        # Initialisation de la fenêtre principale
        self.root = tk.Tk()
        self.root.title("Validation")
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        if screen_width == 480 and screen_height == 800:
            # Plein écran
            self.root.attributes("-fullscreen", True)
        else:
            self.root.geometry("480x800")
        self.root.configure(bg="white")  # Couleur de fond

        # Création des widgets
        self.create_widgets()

        # Lancer la boucle principale
        self.root.mainloop()

    def create_widgets(self):
        # Titre
        title_label = tk.Label(
            self.root,
            text="Détails de la Validation",
            font=("Arial", 16, "bold"),
            bg="white",
            fg="black",
        )
        title_label.pack(pady=10)

        # Affichage de la couleur
        color_label = tk.Label(
            self.root,
            text=f"Couleur : {self.color}",
            font=("Arial", 14),
            bg=self.color,  # Utilisation de la couleur fournie
            fg="white" if self.color.lower() != "white" else "black",
            width=30,  # Largeur fixe
            anchor="w",  # Alignement à gauche
            padx=10,
        )
        color_label.pack(pady=5)

        # Affichage du script
        script_label = tk.Label(
            self.root,
            text=f"Script : {self.script}",
            font=("Arial", 14),
            bg="white",
            fg="black",
            anchor="w",
            padx=10,
        )
        script_label.pack(pady=5)

        # Bouton de validation
        validate_button = tk.Button(self.root,
                                    text="Valid",
                                    command=self.on_validate)
        validate_button.pack(pady=10)

        reload_button = tk.Button(self.root,
                                  text="Reload",
                                  command=self.on_reload)
        reload_button.pack(pady=20)

    def on_validate(self):
        print("Validation effectuée.")
        self.root.destroy()

    def on_reload(self):
        self.reload = True
        self.root.destroy()


class ScoreApp:
    def __init__(self, color):
        self.root = tk.Tk()
        self.root.title("Score")
        self.color = color
        self.score = 0
        self.is_match = False
        self.is_au = False
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        if screen_width == 480 and screen_height == 800:
            # Plein écran
            self.root.attributes("-fullscreen", True)
        else:
            self.root.geometry("480x800")
        self.root.configure(bg=self.color)  # Couleur de fond
        # Création de la frame
        self.create_frame()

    def create_frame(self):
        frame = tk.Frame(self.root, bg="lightgray", relief="solid", bd=2)
        frame.pack(expand=True, fill="both", padx=20, pady=20)

        self.display_score = tk.StringVar()
        self.display_score.set("0")

        self.zero_label = tk.Label(
            frame,
            textvariable=self.display_score,
            font=("Arial", 120, "bold"),
            bg="lightgray",
            fg="black",
        )
        self.zero_label.pack(expand=True)

    def update_au(self):
        if self.is_au:
            self.root.configure(bg="red")
            if self.is_match:
                root = tk.Toplevel()
                gif_path = os.path.join(
                    get_package_share_directory("opossum_ihm"),
                    "images",
                    "boulette.gif",
                )
                lbl = ImageLabel(root)
                lbl.pack()
                lbl.load(gif_path)
                root.after(3000, root.destroy)
                root.mainloop()
        else:
            self.root.configure(bg=self.color)
        self.root.after(500, self.update_au)

    def update_score(self):
        """Met à jour le score toutes les 500ms"""
        # self.score = score  # Incrémentation du score
        logger = get_logger("opossum_ihm")
        logger.info(f"Score: {self.score}")
        self.display_score.set(str(self.score))  # Mise à jour du texte
        self.zero_label.update_idletasks()

        # Planifier la prochaine mise à jour dans 500ms
        self.root.after(500, self.update_score)


class ImageLabel(tk.Label):
    """
    A Label that displays images, and plays them if they are gifs
    :im: A PIL Image instance or a string filename
    """

    def load(self, im):
        if isinstance(im, str):
            im = Image.open(im)
        frames = []

        try:
            for i in count(1):
                frames.append(ImageTk.PhotoImage(im.copy()))
                im.seek(i)
        except EOFError:
            pass
        self.frames = cycle(frames)

        try:
            self.delay = im.info["duration"]
        except:
            self.delay = 100

        if len(frames) == 1:
            self.config(image=next(self.frames))
        else:
            self.next_frame()

    def unload(self):
        self.config(image=None)
        self.frames = None

    def next_frame(self):
        if self.frames:
            self.config(image=next(self.frames))
            self.after(self.delay, self.next_frame)


class GUI:
    def __init__(self):
        self.reload = False
        self.initialized = False

    def run_color(self):
        self.color_app = ColorChoiceApp()
        self.reload = self.color_app.reload

    def get_color(self):
        return self.color_app.selected_color

    def run_script(self):
        assert self.color_app.selected_color is not None
        self.img_app = ImageApp(self.color_app.selected_color)
        self.reload = self.img_app.reload

    def get_script(self):
        return self.img_app.selected_script

    def run_validation(self):
        assert self.color_app.selected_color is not None
        assert self.img_app.selected_script is not None
        self.validation_app = ValidationApp(
            self.color_app.selected_color,
            self.img_app.selected_script
        )
        self.reload = self.validation_app.reload

    def run_score(self):
        self.score_app = ScoreApp(self.color_app.selected_color)
