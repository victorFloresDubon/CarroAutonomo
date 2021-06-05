# More info: https://jorgecasas.github.io/2017/08/22/autonomous-rc-car-construyendo-un-coche-autonomo
#
# Importamos librerias RPi.GPIO (entradas/salidas GPIO de Raspberry Pi) y time (para sleeps, etc...)
# Requiere previamente instalarla (pip install RPi.GPIO)
import RPi.GPIO as GPIO
import time
import io
import socket
import struct
import picamera
import threading

# Configure Raspberry Pi GPIO in BCM mode
GPIO.setmode(GPIO.BCM)



# Config vars. These IP and ports must be available in server firewall
log_enabled = True
server_ip = '192.168.0.2'
server_port_ultrasonic = 7692
server_port_camera = 7690
server_port_mensaje = 7691

# Camera configuration
image_width = 640
image_height = 480
image_fps = 10
recording_time = 600

# Ultrasonic sensor configuration
ultrasonic_sensor_enabled = True
# Motor sensor configuration
motor_enabled = True

# Definition of GPIO pins in Raspberry Pi 3 (GPIO pins schema needed!)
#   18 - Trigger (output)
#   24 - Echo (input)
GPIO_ultrasonic_trigger = 18
GPIO_ultrasonic_echo = 24
# GPIO pins configuration for DC Motor 1 
MOTOR_01_GPIO_FORWARD = 17
MOTOR_01_GPIO_BACKWARD = 27

# Clase para manejar los mensajes enviados por el servidor de OpenCV
class MensajeListener():
    
    def __init__(self, host, port):
        print( '?? init-mensajeListener' +  str( host ) + ':' + str( port ))
        self.host = host
        self.port = port
        # Conecta al socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect( (host, port) )
    
    """
    Examina los mensajes enviados por el servidor de acuerdo a lo observado por la cámara o
        el sensor ultrasónico, los cuales se manejarán de la siguiente forma
    D = Directo
    R = Retroceso
    P = Parar
    F = Finalizar
    """
    def get_mensaje(self):
        # Reciviremos un mensaje de 8 bytes de longitud
        mensaje = self.sock.recv(8)
        mensaje = mensaje.decode("utf-8")
        # Retornamos el mensaje decodificado
        return mensaje
# Clase para manejar los motores de movimiento

class StreamMotores():

    def Stop(self):
        GPIO.output( MOTOR_01_GPIO_FORWARD, GPIO.LOW)
        GPIO.output( MOTOR_01_GPIO_BACKWARD, GPIO.LOW)
        print( '=== Parar \r' )

    def Forward(self):
        GPIO.output( MOTOR_01_GPIO_BACKWARD, GPIO.LOW)
        GPIO.output( MOTOR_01_GPIO_FORWARD, GPIO.HIGH)
        print('+++ Avanzar \r')

    def Backward(self): 
        GPIO.output( MOTOR_01_GPIO_FORWARD, GPIO.LOW)
        GPIO.output( MOTOR_01_GPIO_BACKWARD, GPIO.HIGH)
        print('--- Retroceder \r')
        
    def __init__(self):
        print( '++ Iniciando control de movimiento ' )
        time.sleep(1.5) 
        # Raspberry Pi and L298N bridge must have the same GROUND pins interconnected
        # Choose BCM numbering schemes for GPIO pins in Raspberry Pi  
        #GPIO.setmode(GPIO.BCM)  

        # Enabling GPIO configured pins as OUTPUT - Also, create objects PWM on ports defined with 50 Hertz configuration
        GPIO.setup( MOTOR_01_GPIO_FORWARD, GPIO.OUT)
        GPIO.setup( MOTOR_01_GPIO_BACKWARD, GPIO.OUT)

        try:
            ml = MensajeListener(server_ip, server_port_mensaje)
            print( '++ Esperando mensaje' )

            while True:
        #         Examina los mensajes enviados por el servidor de acuerdo a lo observado por la cámara o
        #         el sensor ultrasónico, los cuales se manejarán de la siguiente forma
        #         D = Directo
        #         R = Retroceso
        #         P = Parar
        #         F = Finalizar
                char = ml.get_mensaje()
                if char == 'F':
                    # Ending
                    break
                # Parar
                elif char == 'P':
                    self.Stop()
                # Directo
                elif char == 'D':
                    self.Forward()
                # Retoceso
                elif char == 'R':
                    self.Backward()
                elif char == '':
                    print( '¡¡ SIN MENSAJE !! ' )

        finally:

            # Stop GPIO and cleaning pins
#             GPIO.output( MOTOR_01_GPIO_FORWARD, GPIO.LOW)
#             GPIO.output( MOTOR_01_GPIO_BACKWARD, GPIO.LOW)
#             GPIO.cleanup()

            print ('-- FIN MOTORES')


# Class to handle the ultrasonic sensor stream in client
class StreamClientUltrasonic():

    def measure(self):
        # Measure distance from ultrasonic sensor. Send a trigger pulse
        GPIO.output( GPIO_ultrasonic_trigger, True )
        time.sleep( 0.00001 )
        GPIO.output( GPIO_ultrasonic_trigger, False )
        
        # Get start time
        start = time.time()

        # Wait to receive any ultrasound in sensor (echo)
        while GPIO.input( GPIO_ultrasonic_echo ) == 0:
            start = time.time()

        # We have received the echo. Wait for its end, getting stop time
        while GPIO.input( GPIO_ultrasonic_echo ) == 1:
            stop = time.time()

        # Calculate time difference. Sound has gone from trigger to object and come back to sensor, so 
        # we have to divide between 2. Formula: Distance = ( Time elapsed * Sound Speed ) / 2 
        time_elapsed = stop-start
        distance = (time_elapsed * 34300) / 2

        return distance

  
    def __init__(self):

        # Connect a client socket to server_ip:server_port_ultrasonic
        print( '+ Trying to connect to ultrasonic streaming server in ' + str( server_ip ) + ':' + str( server_port_ultrasonic ) );

        # Configure GPIO pins (trigger as output, echo as input)
        GPIO.setup( GPIO_ultrasonic_trigger, GPIO.OUT )
        GPIO.setup( GPIO_ultrasonic_echo, GPIO.IN )

        # Set output GPIO pins to False
        GPIO.output( GPIO_ultrasonic_trigger, False ) 

        # Create socket and bind host
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect( ( server_ip, server_port_ultrasonic ) )

        try:
            while True:
                # Measure and send data to the host every 0.5 sec, 
                # pausing for a while to no lock Raspberry Pi processors
                distance = self.measure()
                if log_enabled: print( "Ultrasonic sensor distance: %.1f cm" % distance )
                client_socket.send( str( distance ).encode('utf-8') )
                time.sleep( 0.5 )

        finally:
            # Ctrl + C to exit app (cleaning GPIO pins and closing socket connection)
            print( 'Ultrasonic sensor connection finished!' );
            client_socket.close()
            GPIO.cleanup()


# Class to handle the jpeg video stream in client
class StreamClientVideocamera():
  
    def __init__(self):

        # Connect a client socket to server_ip:server_port_camera
        print( '+ Trying to connect to videocamera streaming server in ' + str( server_ip ) + ':' + str( server_port_camera ) );

        # create socket and bind host
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((server_ip, server_port_camera))
        connection = client_socket.makefile('wb')

        try:
            with picamera.PiCamera() as camera:
                camera.resolution = (image_width, image_height)
                camera.framerate = image_fps

                # Give 2 secs for camera to initilize
                time.sleep(2)                       
                start = time.time()
                stream = io.BytesIO()
                
                # send jpeg format video stream
                for foo in camera.capture_continuous(stream, 'jpeg', use_video_port = True):
                    connection.write(struct.pack('<L', stream.tell()))
                    connection.flush()
                    stream.seek(0)
                    connection.write(stream.read())
                    if time.time() - start > recording_time:
                        break
                    stream.seek(0)
                    stream.truncate()
            connection.write(struct.pack('<L', 0))

        finally:
            connection.close()
            client_socket.close()
            print( 'Videocamera stream connection finished!' );

# Class to handle the different threads in client 
class ThreadClient():

    # Client thread to handle the video
    def client_thread_camera(host, port):
        print( '+ Starting videocamera stream client connection to ' + str( host ) + ':' + str( port ) )
        StreamClientVideocamera()

    # Client thread to handle ultrasonic distances to objects
    def client_thread_ultrasonic(host, port):
        print( '+ Starting ultrasonic stream client connection to ' + str( host ) + ':' + str( port ) )
        StreamClientUltrasonic()
    
    def client_thread_controlMovimiento(host, port):
        print('+ Iniciando control de movimiento ' + str( host ) + ':' + str( port ))
        StreamMotores()

    print( '+ Starting client - Logs ' + ( log_enabled and 'enabled' or 'disabled'  ) )

    if ultrasonic_sensor_enabled:
        thread_ultrasonic = threading.Thread( name = 'thread_ultrasonic', target = client_thread_ultrasonic, args = ( server_ip, server_port_ultrasonic ) )
        thread_ultrasonic.start()

    thread_videocamera = threading.Thread( name = 'thread_videocamera', target = client_thread_camera, args = ( server_ip, server_port_camera ) )
    thread_videocamera.start()
    
    if motor_enabled:
         thread_controlMovimiento = threading.Thread( name = 'thread_controlMovimiento', target = client_thread_controlMovimiento, args = (server_ip, server_port_mensaje ) )
         thread_controlMovimiento.start()

    

# Starting thread client handler
if __name__ == '__main__':
    ThreadClient()
