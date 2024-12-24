import tkinter as tk
from PIL import Image, ImageTk
import os
from pathlib import Path

class ColorChoiceApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Choisir une couleur")
        self.root.geometry("480x800")

        self.label = tk.Label(root, text="Choisissez une couleur :")
        self.label.pack(pady=10)

        self.button_yellow = tk.Button(root,
                                       text="Jaune",
                                       bg="yellow",
                                       command=lambda: self.chs_clr("jaune"))
        self.button_yellow.pack(side=tk.LEFT, padx=20)

        self.button_blue = tk.Button(root,
                                     text="Bleu",
                                     bg="blue",
                                     fg="white",
                                     command=lambda: self.chs_clr("bleu"))
        self.button_blue.pack(side=tk.LEFT, padx=20)

        self.selected_color = None

        # Bouton pour quitter
        self.button_quit = tk.Button(root, text="Quitter", command=root.quit)
        self.button_quit.pack(pady=10)


    def chs_clr(self, color):
        self.selected_color = color
        self.root.destroy()
        self.open_image_window()

    def open_image_window(self):
        image_window = tk.Tk()
        ImageApp(image_window, self.selected_color)
        image_window.mainloop()


class ImageApp:
    def __init__(self, root, selected_color):
        self.root = root
        self.root.title("Cliquez sur l'image")
        self.root.geometry("480x800")
        self.color = selected_color 

        current_dir = Path(__file__).resolve().parent
        image_path = os.path.join(current_dir, "plateau.png")
        self.img = Image.open(image_path)
        # Pivoter l'image de 90° dans le sens des aiguilles d'une montre
        self.img = self.img.rotate(90, expand=True)
        # Dimensions de la fenêtre
        window_width = 480
        window_height = 800
        # Dimensions de l'image après la rotation
        img_width, img_height = self.img.size
        # Calculer les facteurs de redimensionnement pour la largeur et la hauteur
        width_factor = window_width / img_width
        height_factor = window_height / img_height
        # Choisir le plus petit facteur pour conserver le ratio
        scale_factor = min(width_factor, height_factor)
        # Redimensionner l'image en maintenant le ratio
        new_width = int(img_width * scale_factor)
        new_height = int(img_height * scale_factor)
        # Redimensionner l'image
        self.img = self.img.resize((new_width, new_height))
        # Convertir l'image en format compatible avec Tkinter
        self.tk_img = ImageTk.PhotoImage(self.img)
        # Créer un canevas pour afficher l'image
        self.canvas = tk.Canvas(root, width=window_width, height=window_height)
        self.canvas.pack()
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_img)
        # Bind pour obtenir les coordonnées du clic
        self.canvas.bind("<Button-1>", self.get_coordinates)
        # Label pour afficher les coordonnées
        self.label = tk.Label(root, text=f"Couleur choisie: {self.color}")
        self.label.pack(pady=10)

        # Bouton pour quitter
        self.button_quit = tk.Button(root, text="Quitter", command=root.quit)
        self.button_quit.pack(pady=10)


    def get_coordinates(self, event):
        x, y = event.x, event.y
        self.label.config(text=f"Couleur: {self.color} | x={x}, y={y})")
        print(f"Couleur choisie: {self.color}, Coordonnées: ({x}, {y})")


def main():
    root = tk.Tk()
    app = ColorChoiceApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()
