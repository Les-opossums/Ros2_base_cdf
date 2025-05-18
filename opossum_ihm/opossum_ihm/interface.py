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
            self.root,
            text="Reload",
            bg="red",
            command=self.reload_ihm,
            width=10,
            height=5,
        )
        self.button_reload.pack(pady=25)

        # Bouton pour quitter
        # self.button_quit = tk.Button(self.root,
        #                              text="Quit",
        #                              command=self.root.destroy,
        #                              width=10,
        #                              height=5,
        #                              )
        # self.button_quit.pack(pady=20)
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
        self.script_zones = {
            1: (0, 515, 105, 620),
            2: (190, 545, 300, 650),
            3: (330, 330, 480, 450),
            4: (330, 210, 480, 320),
            5: (190, 0, 300, 110),
            6: (0, 30, 105, 140)
        }
        self.zone_rects = {}

        for script_num, (x1, y1, x2, y2) in self.script_zones.items():
            rect = self.canvas.create_rectangle(x1, y1, x2, y2, fill="grey",
                                                outline="black"
                                                )
            text = self.canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2,
                                           text=str(script_num),
                                           fill="white",
                                           font=("Helvetica", 20, "bold")
                                           )
            self.zone_rects[script_num] = (rect, text)
        # Bind pour obtenir les coordonnées du clic
        self.canvas.bind("<Button-1>", self.get_coordinates)
        # Label pour afficher les coordonnées
        self.label = tk.Label(self.root, text=f"Color: {self.color}")
        self.label.pack(pady=10)

        self.button_reload = tk.Button(
            self.root,
            text="Reload",
            bg="red",
            command=self.reload_ihm,
            height=5,
            width=10
        )
        self.button_reload.pack(padx=0, pady=10)
        self.button_reload.place(x=100, y=700)

        # Bouton pour quitter
        self.button_valid = tk.Button(
            self.root,
            text="Valid",
            bg="green",
            command=self.verify_selection,
            height=5,
            width=10
        )
        self.button_valid.pack(padx=20, pady=10)
        self.button_valid.place(x=300, y=700)
        self.root.mainloop()

    def get_coordinates(self, event):
        x, y = event.x, event.y
        print(f"Couleur choisie: {self.color}, Coordonnées: ({x}, {y})")

        selected = 0
        for script_num, (x1, y1, x2, y2) in self.script_zones.items():
            if x1 <= x <= x2 and y1 <= y <= y2:
                selected = script_num
                break

        self.selected_script = selected

        # Update colors of all zones
        for script_num, (rect_id, _) in self.zone_rects.items():
            color = "green" if script_num == self.selected_script else "grey"
            self.canvas.itemconfig(rect_id, fill=color)

        self.label.config(text=f"Script:{self.selected_script} | x={x}, y={y}")

    def reload_ihm(self):
        self.root.destroy()
        self.reload = True

    def verify_selection(self):
        if self.selected_script == 0:
            self.show_temp_popup("Please select a script!")
        elif self.selected_script % 2 == 0 and self.color == "yellow":
            self.show_temp_popup("Invalid color for this script!")
        elif self.selected_script % 2 != 0 and self.color == "blue":
            self.show_temp_popup("Invalid color for this script!")
        else:
            self.root.destroy()

    def show_temp_popup(self, message):
        popup = tk.Toplevel(self.root)
        popup.title("Warning")
        popup.geometry("250x100")
        popup.configure(bg="white")
        popup.transient(self.root)  # lie le popup à la fenêtre principale
        popup.grab_set()  # empêche l'interaction avec la fenêtre principale

        label = tk.Label(popup, text=message, bg="white",
                         fg="red", font=("Helvetica", 12, "bold"))
        label.pack(expand=True, padx=10, pady=10)

        # Ferme automatiquement la fenêtre après 1000 ms
        self.root.after(1000, popup.destroy)


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
                                    command=self.on_validate,
                                    bg="green",
                                    height=5,
                                    width=10
                                    )
        validate_button.pack(pady=10)

        reload_button = tk.Button(self.root,
                                  text="Reload",
                                  command=self.on_reload,
                                  bg="red",
                                  height=5,
                                  width=10
                                  )
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
        self.comm_state = True
        self.lidar_pos_x = 0.
        self.lidar_pos_y = 0.
        self.lidar_pos_z = 0.
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
        self.position_text = tk.StringVar()
        self.position_text.set(f" X: {self.lidar_pos_x} "
                               f" Y: {self.lidar_pos_y} "
                               f" T: {self.lidar_pos_z} "
                               )

        self.zero_label = tk.Label(
            frame,
            textvariable=self.display_score,
            font=("Arial", 120, "bold"),
            bg="lightgray",
            fg="black",
        )
        self.zero_label.pack(expand=True)

        self.position_label = tk.Label(
            frame,
            textvariable=self.position_text,
            font=("Arial", 20),
            bg="lightgray",
            fg="black",
            anchor="s"
        )
        self.position_label.pack(side="bottom", pady=10)

    def update_au(self):
        if self.is_au:
            self.root.configure(bg="red")
            # if self.is_match:
            if True:
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
        elif not self.comm_state:
            self.root.configure(bg="orange")
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

    def update_position(self):
        """Met à jour la position toutes les 500ms"""
        self.position_text.set(f"X: {self.lidar_pos_x}"
                               f"Y: {self.lidar_pos_y}"
                               f"T: {self.lidar_pos_z}"
                               )
        self.position_label.update_idletasks()

        # Planifier la prochaine mise à jour dans 500ms
        self.root.after(500, self.update_position)


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
