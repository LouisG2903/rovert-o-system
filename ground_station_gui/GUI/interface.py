import customtkinter as ctk

# clase padre (window)
class App(ctk.CTk):
    # parametros de la clase
    def __init__(self, title: str="Window", size: tuple=(512, 512)):
        # inicializacion de la aplicacion
        super().__init__() # fg_color
        self.title(title)
        self.geometry(f"{size[0]}x{size[1]}")
        self.resizable(False, False)
        self.fonts = [ctk.CTkFont(family="Noto Sans Mono", size=16, weight="bold"), ctk.CTkFont(family="Noto Sans Mono", size=14, weight="normal")]
        
        #instancia de seccion (frame) de parametros del rover
        self.rover_params = self.RoverParams(master=self)
        self.rover_params.place(relx=0.05, rely=0.6, relwidth=0.55, relheight=0.3)
        self.rover_params.grid_columnconfigure((0, 1, 2), weight=1)
        self.rover_params.grid_rowconfigure((0, 1, 2), weight=1)
        
        # instancia de seccion (frame) de instrucciones
        self.instructions = self.Instructions(master=self)
        self.instructions.place(relx=0.65, rely=0.6, relwidth=0.3, relheight=0.3)

    # clase de seccion de instrucciones (frame)
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
            self.entry_distance = ctk.CTkEntry(master=self, corner_radius=8, placeholder_text="Ingresa distancia...", font=master.fonts[1])
            self.entry_distance.place(anchor="nw", relx=0.1, rely=0.2, relwidth=0.5, relheight=0.1)
            
            # direccion
            self.label_direction = ctk.CTkLabel(master=self, text="Dirección:", font=master.fonts[0])
            self.label_direction.place(anchor="nw", relx=0.1, rely=0.35)
            self.entry_direction = ctk.CTkEntry(master=self, corner_radius=8, placeholder_text="Ingresa dirección...", font=master.fonts[1])
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
            self.label_state = ctk.CTkLabel(master=self, textvariable=self.text_state, font=master.fonts[1])
            self.label_state.place(anchor="nw", relx=0.15, rely=0.85)

        # evento del boton enviar
        def button_send_pressed(self):
            pass

        # evento del boton detener
        def button_stop_pressed(self):
            pass
    
    # clase de seccion de parametros del rover (frame)    
    class RoverParams(ctk.CTkFrame):
        #parametros de la clase
        def __init__(self, master):
            super().__init__(master, corner_radius=32)
            # variables de texto
            self.text_env_temp = ctk.StringVar(value="(min, max)")
            self.text_bat_temp = ctk.StringVar(value="(min, max)")
            
            #variables de valor
            self.val_env_temp = ctk.StringVar(value="null")
            self.val_bat_temp = ctk.StringVar(value="null")
            
            # parametro
            self.label_param = ctk.CTkLabel(master=self, text="Parámetro", font=master.fonts[0])
            self.label_param.grid(row=0, column=0)
            
            # actual
            self.label_actual = ctk.CTkLabel(master=self, text="Actual", font=master.fonts[0])
            self.label_actual.grid(row=0, column=1)
            
            # normal
            self.label_normal = ctk.CTkLabel(master=self, text="Normal", font=master.fonts[0])
            self.label_normal.grid(row=0, column=2)
            
            # temperatura de ambiente
            self.label_env_temp = ctk.CTkLabel(master=self, text="T. ambiente [\u00b0C]", font=master.fonts[1])
            self.label_env_temp.grid(row=1, column=0)
            self.label_env_temp_current = ctk.CTkLabel(master=self, textvariable=self.val_env_temp, font=master.fonts[1])
            self.label_env_temp_current.grid(row=1, column=1)
            self.label_env_temp_normal = ctk.CTkLabel(master=self, textvariable=self.text_env_temp, font=master.fonts[1])
            self.label_env_temp_normal.grid(row=1, column=2)
            
            # temperatura de bateria
            self.label_bat_temp = ctk.CTkLabel(master=self, text="T. batería [\u00b0C]", font=master.fonts[1])
            self.label_bat_temp.grid(row=2, column=0)
            self.label_bat_temp_current = ctk.CTkLabel(master=self, textvariable=self.val_bat_temp, font=master.fonts[1])
            self.label_bat_temp_current.grid(row=2, column=1)
            self.label_bat_temp_normal = ctk.CTkLabel(master=self, textvariable=self.text_bat_temp, font=master.fonts[1])
            self.label_bat_temp_normal.grid(row=2, column=2)
