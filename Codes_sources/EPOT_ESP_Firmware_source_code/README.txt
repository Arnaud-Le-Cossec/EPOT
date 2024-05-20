READ ME
-------------Comment compiler ce projet ------------
1. ouvrir VScode.
2. Installer PlatformIO.
3. Dans "PlatformIO Home", créer un nouveau projet.
4. Entrer un nom de projet, sélectionner la carte [ESP32 Wrover DevKit] avec Arduino framework.
5. Copier le contenu de "main.cpp" dans le "main" du projet.
6. Copier le fichier "EPOT_lib.h" dans le dossier "src" du project.
7. Dans le fichier "platform.ini", changer ou ajouter les lignes suivantes :
	monitor_speed = 9600
	board_upload.flash_size = 8MB
	board_build.partitions = custom_partition.csv
Le fichier "platform.ini" doit désormais ressembler à celui fourni dans ce dossier
8. Dans la racine du projet, copier le fichier "custom_partition.csv".
9. Depuis "PlatformIO Home", cliquer sur "libraries" pour installer les librairies requises

Le fichier compilé en .elf et .bin est disponible dans le dossier "buid"
----------------- Notes -----------------------
Avant de compiler, il faut changer les paramètres de la base de données dans "EPOT_lib.h" :
#define DATABASE_API_KEY "entrez votre API de base de données"
#define DATABASE_URL "entrez votre URL de base de données" 
#define DATABASE_USER_EMAIL "user@email.com"
#define DATABASE_USER_PASSWORD "PASSWORD123"
