import tkinter as tk
from PIL import Image, ImageTk
from rclpy.node import Node
import os
from cdf_msgs.srv import Init
from pathlib import Path
import rclpy 

class ColorChoiceApp():
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Choisir une couleur")

        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        if (screen_width==480 and screen_height==800):
            # Plein écran
            self.root.attributes("-fullscreen", True)
        else :
            self.root.geometry("480x800")
        self.label = tk.Label(self.root, text="Choisissez une couleur :")
        self.label.pack(pady=10)

        self.button_yellow = tk.Button(self.root,
                                       text="Yellow",
                                       bg="yellow",
                                       command=lambda: self.chs_clr("yellow"),
                                       width=20,
                                       height=10
                                    )
        self.button_yellow.pack(padx=20)

        self.button_blue = tk.Button(self.root,
                                     text="Blue",
                                     bg="blue",
                                     command=lambda: self.chs_clr("blue"),
                                     width=20,
                                     height=10
                                    )
        self.button_blue.pack(padx=20)

        self.selected_color = None

        self.button_reload = tk.Button(self.root, text="Reload", command=self.reload_ihm)
        self.button_reload.pack(pady=10)

        # Bouton pour quitter
        self.button_quit = tk.Button(self.root, text="Quit", command=self.root.destroy)
        self.button_quit.pack(pady=20)
        self.root.mainloop()


    def chs_clr(self, color):
        self.selected_color = color
        #self.update_parameters()
        self.root.destroy()
        #self.open_image_window()

    def open_image_window(self):
        image_window = tk.Tk()
        ImageApp(image_window, self.selected_color)
        image_window.mainloop()

    # def update_parameters(self):
    #     client = self.create_client(Init, 'set_parameters')
    #     request = Init.Request()
    #     request.team_color = self.selected_color
    #     future = client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)

    def reload_ihm(self):
        self.root.destroy()
        self.__init__()


class ImageApp():
    def __init__(self, selected_color):
        self.selected_script = 100
        self.selected_color = selected_color

        self.root = tk.Tk()
        self.root.title("Choose your script")
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        
        if (screen_width==480 and screen_height==800):
            # Plein écran
            self.root.attributes("-fullscreen", True)
        else :
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
        self.canvas = tk.Canvas(self.root, width=new_width, height=new_height)
        self.canvas.pack()
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_img)
        # Bind pour obtenir les coordonnées du clic
        self.canvas.bind("<Button-1>", self.get_coordinates)
        # Label pour afficher les coordonnées
        self.label = tk.Label(self.root, text=f"Color: {self.color}")
        self.label.pack(pady=10)
        
        self.button_reload = tk.Button(self.root, 
                                       text="Reload", 
                                       command=self.reload_ihm,
                                       height=3,
                                       width=10
                                       )
        self.button_reload.pack(padx=0,pady=10)
        self.button_reload.place(x=100, y=760)

        # Bouton pour quitter
        self.button_quit = tk.Button(self.root, 
                                     text="Quit", 
                                     command=self.root.destroy,
                                     height=3,
                                     width=10
                                     )
        self.button_quit.pack(padx=20,pady=10)
        self.button_quit.place(x=300, y=760)
        self.root.mainloop()

    def get_coordinates(self, event):
        x, y = event.x, event.y
        self.label.config(text=f"Couleur: {self.color} | x={x}, y={y})")
        print(f"Couleur choisie: {self.color}, Coordonnées: ({x}, {y})")
        self.selected_script = 1

    def reload_ihm(self):
        self.root.destroy()
        self.__init__(self.selected_color)

    #def read_parameters(self):
    #    client = self.create_client(Init, 'get_parameters')
    #    request = Init.Request()
    #    future = client.call_async(request)
    #    rclpy.spin_until_future_complete(self, future)
    #    return future.result().team_color

class GUI:
    def __init__(self):
        self.color_app = ColorChoiceApp()
        self.get_logger().info("Color selected: " + self.color_app.selected_color)
        self.img_app = ImageApp(self.color_app.selected_color)
        self.get_logger().info("Script selected: " + str(self.img_app.selected_script))

    def get_color(self):
        return self.color_app.selected_color

    def get_script(self):
        return self.img_app.selected_script

# if __name__ == '__main__':
#     main()
