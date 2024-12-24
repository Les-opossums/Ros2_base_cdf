import tkinter as tk
from PIL import Image, ImageTk
import os
from pathlib import Path

class ColorChoiceApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Choisir une couleur")
        self.root.attributes('-fullscreen', True)

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
        self.root.attributes('-fullscreen', True)
        self.color = selected_color 

        current_dir = Path(__file__).resolve().parent
        image_path = os.path.join(current_dir, "plateau.png")
        self.img = Image.open(image_path)
        self.img = self.img.resize((915, 611))
        self.tk_img = ImageTk.PhotoImage(self.img)

        self.canvas = tk.Canvas(root, width=915, height=611)
        self.canvas.pack()

        # Afficher l'image sur le canevas
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
