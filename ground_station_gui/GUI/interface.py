import customtkinter as ctk

# === clase padre (window) ===
class App(ctk.CTk):
    # parametros de la clase
    def __init__(self, title: str="Window", size: tuple=(512, 512)):
        # inicializacion de la aplicacion
        super().__init__() # fg_color
        self.title(title)
        self.geometry(f"{size[0]}x{size[1]}")
        self.resizable(False, False)
        self.grid_columnconfigure((0, 1, 2), weight=1)
        self.grid_rowconfigure((0, 1, 2), weight=1)
        
        # fuentes de la interfaz
        self.fonts = [ctk.CTkFont(family="Noto Sans Mono", size=24, weight="bold"), ctk.CTkFont(family="Noto Sans Mono", size=24, weight="normal"), ctk.CTkFont(family="Noto Sans Mono", size=24, slant="italic")]
        
        # instancia de seccion (frame) de parametros del rover
        self.rover_params = self.RoverParams(master=self)
        self.rover_params.grid(row=0, column=2, sticky="nsew", padx=16, pady=16)
        self.rover_params.grid_columnconfigure((0, 1, 2), weight=1)
        self.rover_params.grid_rowconfigure((0, 1, 2), weight=1)
        
        # instancia de seccion (frame) de instrucciones
        self.instructions = self.Instructions(master=self)
        self.instructions.grid(row=1, column=2, rowspan=2, sticky="nsew", padx=16, pady=16)
        
        # instancia de seccion (frame) de parametros del mapa
        self.map_params = self.MapParams(master=self)
        self.map_params.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=16, pady=16)
        self.map_params.grid_columnconfigure((0, 1, 2), weight=1)
        self.map_params.grid_rowconfigure((0, 1, 2, 3), weight=1)
        
        # instancia de seccion (frame) de mapa
        self.map = self.Map(master=self)
        self.map.grid(row=0, column=0, rowspan=2, columnspan=2, sticky="nsew", padx=16, pady=16)

    # === clase de seccion de instrucciones (frame) ===
    class Instructions(ctk.CTkFrame):
        # parametros de la clase
        def __init__(self, master):
            # inicializacion del frame
            super().__init__(master, corner_radius=32)
            
            # variables de texto
            self.text_state = ctk.StringVar(value="Detenido")
            
            # distancia
            self.label_distance = ctk.CTkLabel(master=self, text="Distancia [m]:", font=master.fonts[0])
            self.label_distance.place(anchor="nw", relx=0.1, rely=0.1)
            self.entry_distance = ctk.CTkEntry(master=self, corner_radius=8, placeholder_text="Ingresa distancia...", font=master.fonts[2])
            self.entry_distance.place(anchor="nw", relx=0.1, rely=0.2, relwidth=0.5, relheight=0.1)
            
            # direccion
            self.label_direction = ctk.CTkLabel(master=self, text="Dirección [\u00b0]:", font=master.fonts[0])
            self.label_direction.place(anchor="nw", relx=0.1, rely=0.35)
            self.entry_direction = ctk.CTkEntry(master=self, corner_radius=8, placeholder_text="Ingresa dirección...", font=master.fonts[2])
            self.entry_direction.place(anchor="nw", relx=0.1, rely=0.45, relwidth=0.5, relheight=0.1)
            
            # boton envio
            self.button_send = ctk.CTkButton(master=self, corner_radius=8, text="Enviar", font=master.fonts[0], command=self.button_send_pressed)
            self.button_send.place(anchor="nw", relx=0.1, rely=0.6, relwidth=0.35, relheight=0.1)
            
            # boton detener
            self.button_stop = ctk.CTkButton(master=self, corner_radius=8, text="Detener", font=master.fonts[0], command=self.button_stop_pressed)
            self.button_stop.place(anchor="nw", relx=0.55, rely=0.6, relwidth=0.35, relheight=0.1)
            
            # accion
            self.label_action = ctk.CTkLabel(master=self, text="Acción en proceso:", font=master.fonts[0])
            self.label_action.place(anchor="nw", relx=0.1, rely=0.75)
            self.label_state = ctk.CTkLabel(master=self, textvariable=self.text_state, font=master.fonts[2])
            self.label_state.place(anchor="nw", relx=0.15, rely=0.85)
        
        # funcion de boton de envio
        def button_send_pressed(self):
            pass
        
        # funcion de boton de detener
        def button_stop_pressed(self):
            pass
            
    # === clase de seccion de parametros del rover (frame) ===
    class RoverParams(ctk.CTkFrame):
        # parametros de la clase
        def __init__(self, master):
            # inicializacion del frame
            super().__init__(master, corner_radius=32)
            
            # variables de texto
            self.text_env_temp = ctk.StringVar(value="(min, max)") # === modificar por valores de electronica ===
            self.text_bat_temp = ctk.StringVar(value="(min, max)") # === modificar por valores de electronica ===
            
            #variables de valor
            self.val_env_temp = ctk.StringVar(value="null")
            self.val_bat_temp = ctk.StringVar(value="null")
            
            # temperatura
            self.label_param = ctk.CTkLabel(master=self, text="Temperatura", font=master.fonts[0])
            self.label_param.grid(row=0, column=0)
            
            # actual
            self.label_current = ctk.CTkLabel(master=self, text="Actual", font=master.fonts[0])
            self.label_current.grid(row=0, column=1)
            
            # normal
            self.label_normal = ctk.CTkLabel(master=self, text="Normal", font=master.fonts[0])
            self.label_normal.grid(row=0, column=2)
            
            # ambiente
            self.label_env_temp = ctk.CTkLabel(master=self, text="Ambiente [\u00b0C]", font=master.fonts[1])
            self.label_env_temp.grid(row=1, column=0)
            self.label_env_temp_current = ctk.CTkLabel(master=self, textvariable=self.val_env_temp, font=master.fonts[2])
            self.label_env_temp_current.grid(row=1, column=1)
            self.label_env_temp_normal = ctk.CTkLabel(master=self, textvariable=self.text_env_temp, font=master.fonts[1])
            self.label_env_temp_normal.grid(row=1, column=2)
            
            # bateria
            self.label_bat_temp = ctk.CTkLabel(master=self, text="Batería [\u00b0C]", font=master.fonts[1])
            self.label_bat_temp.grid(row=2, column=0)
            self.label_bat_temp_current = ctk.CTkLabel(master=self, textvariable=self.val_bat_temp, font=master.fonts[2])
            self.label_bat_temp_current.grid(row=2, column=1)
            self.label_bat_temp_normal = ctk.CTkLabel(master=self, textvariable=self.text_bat_temp, font=master.fonts[1])
            self.label_bat_temp_normal.grid(row=2, column=2)
        
        # funcion para establecer el valor de ambiente
        def set_ambiente(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_env_temp.set(valor)
        
        # funcion para establecer el valor de bateria
        def set_bateria(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_bat_temp.set(valor)
    
    # === clase de seccion de parametros del mapa (frame) ===        
    class MapParams(ctk.CTkFrame):
        # parametros de la clase
        def __init__(self, master):
            # inicializacion del frame
            super().__init__(master, corner_radius=32)
            
            # variables de valor
            self.val_speed = ctk.StringVar(value="null")
            self.val_angular_ac_x = ctk.StringVar(value="null")
            self.val_angular_ac_y = ctk.StringVar(value="null")
            self.val_angular_ac_z = ctk.StringVar(value="null")
            
            # velocidad
            self.label_speed = ctk.CTkLabel(master=self, text="Velocidad [m/s]:", font=master.fonts[0])
            self.label_speed.grid(row=0, column=0)
            self.label_speed_current = ctk.CTkLabel(master=self, textvariable=self.val_speed, font=master.fonts[2])
            self.label_speed_current.grid(row=0, column=1, sticky="w")
            
            # aceleracion angular
            self.label_angular_ac = ctk.CTkLabel(master=self, text="Aceleración angular [rad/s\u00b2]", font=master.fonts[0])
            self.label_angular_ac.grid(row=1, column=0, columnspan=3)
            
            # eje x
            self.label_angular_ac_x = ctk.CTkLabel(master=self, text="Eje x", font=master.fonts[1])
            self.label_angular_ac_x.grid(row=2, column=0)
            self.label_angular_ac_x_current = ctk.CTkLabel(master=self, textvariable=self.val_angular_ac_x, font=master.fonts[2])
            self.label_angular_ac_x_current.grid(row=3, column=0)
            
            # eje y
            self.label_angular_ac_y = ctk.CTkLabel(master=self, text="Eje y", font=master.fonts[1])
            self.label_angular_ac_y.grid(row=2, column=1)
            self.label_angular_ac_y_current = ctk.CTkLabel(master=self, textvariable=self.val_angular_ac_y, font=master.fonts[2])
            self.label_angular_ac_y_current.grid(row=3, column=1)
            
            # eje z
            self.label_angular_ac_z = ctk.CTkLabel(master=self, text="Eje z", font=master.fonts[1])
            self.label_angular_ac_z.grid(row=2, column=2)
            self.label_angular_ac_z_current = ctk.CTkLabel(master=self, textvariable=self.val_angular_ac_z, font=master.fonts[2])
            self.label_angular_ac_z_current.grid(row=3, column=2)
        
        # funcion para establecer valor de velocidad
        def set_velocidad(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_speed.set(valor)
        
        # funcion para establecer valor de aceleracion en x
        def set_aceleracion_x(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_angular_ac_x.set(valor)
        
        # funcion para establecer valor de aceleracion en y
        def set_aceleracion_y(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_angular_ac_y.set(valor)
        
        # funcion para establecer valor de aceleracion en z
        def set_aceleracion_z(self, valor="null"):
            if type(valor) != "str":
                str(valor)
            self.val_angular_ac_z.set(valor)
            
    class Map(ctk.CTkFrame):
        def __init__(self, master):
            super().__init__(master, corner_radius=32)
