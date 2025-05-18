# opossum_msgs
Ce package contient toutes les définitions de messages utilisés pour le projet
de la Coupe de France de Robotique.


## Règles syntaxiques[^1]

 - Les noms des messages doivent être écrits en `CamelCase`
 - Les champs définits dans les messages doivent être en minuscule et chaque
   mot doit être séparé par un underscore `_`.
 - Le nom des champs doit commencer par un caractère alphanumérique et ne doit
   pas finir par un underscore

[^1]: Aussi consultable à l'adresse suivante : http://design.ros2.org/articles/legacy_interface_definition.html

## Ajouter un message

1. Créer un fichier `<nom du message.msg>` dans le répertoire `msg/` à la racine
   de ce package.
2. Définir le contenu du message, chaque ligne définit un champ d'un type
   particulier.
   Par exemple si l'on souhaite créer un champ `num` du type `int64` il suffit
   d'ajouter la ligne suivante au fichier `.msg`:
      ```
      int64 num
      ```
3. Une fois la définition du message terminé, il faut éditer le fichier
   `CMakeLists.txt` afin d'indiquer que nous avons ajouté un message.
   Ajoutez le nom du fichier de la façon suivante:
      ```
      rosidl_generate_interfaces(${PROJECT_NAME}
         ...
         ...
         "msg/<nom du message.msg>"
         DEPENDENCIES ...
      )
      ```
4. Si le message que vous venez de définir à des dépendances, il faut également
   le renseigner :
      - Ajouter un champ `<depend>dependance</depend>` au fichier `package.xml`
      - Ajouter une ligne `find_package(dependance REQUIRED)` dans le fichier
        `CMakeLists.txt`
      - Ajouter le nom de la dépendance à la fin de la liste des messages du
        package:
           ```
           rosidl_generate_interfaces(${PROJECT_NAME}
            ...
            ...
            DEPENDENCIES ... dependance
           )
           ```

## Dépendances
 - `std_msgs`
 - `geometry_msgs`
