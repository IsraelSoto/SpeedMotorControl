%% ============================================================
%  Lectura y graficado de datos enviados por Arduino
%  Autor: Israel Soto
%  Fecha: 2025
%  Descripción:
%  Este script recibe datos del Arduino mediante el puerto serial.
%  Los datos corresponden a:
%    tiempo [s], posición [rev], velocidad [RPM],
%    velocidad filtrada [RPM], voltaje [V]
%  Posteriormente los grafica y los guarda en un archivo CSV.
% ============================================================

clear; clc; close all;

%% === CONFIGURACIÓN DEL PUERTO SERIAL ========================
port = "COM4";        % Cambia según tu puerto (ej. COM3, /dev/ttyUSB0)
baudrate = 115200;    % Debe coincidir con el del Arduino

arduino = serialport(port, baudrate);
flush(arduino);
configureTerminator(arduino, "LF");

% Ajusta el tiempo máximo de adquisición (igual al del Arduino)
TiempoTotal = 4; % segundos (debe coincidir con el Arduino)
fprintf('Esperando datos por %.1f s...\n', TiempoTotal);

%% === ADQUISICIÓN DE DATOS ===================================
t0 = tic;
data = [];

while toc(t0) <= TiempoTotal + 1
    if arduino.NumBytesAvailable > 0
        try
            line = readline(arduino);
            values = str2double(split(line, ","));
            if numel(values) == 5
                data = [data; values'];
            end
        catch
            % Ignora líneas incompletas o errores de lectura
        end
    end
end

clear arduino;  % Cierra el puerto al terminar
fprintf('Adquisición finalizada. Se recibieron %d muestras.\n', size(data,1));

%% === PROCESAMIENTO ==========================================
if isempty(data)
    error('No se recibieron datos. Verifica conexión o puerto serial.');
end

% Asignación de variables
tiempo = data(:,1);
posicion = data(:,2);
velocidad = data(:,3);
velocidad_filtrada = data(:,4);
voltaje = data(:,5);

%% === GRAFICADO ==============================================
figure('Name','Resultados de la prueba','NumberTitle','off');

subplot(3,1,1);
plot(tiempo, posicion, 'LineWidth', 1.3);
xlabel('Tiempo [s]', 'FontName', 'Times New Roman', 'FontSize', 14);
ylabel('Posición [rev]', 'FontName', 'Times New Roman', 'FontSize', 14);
grid on; title('Posición vs Tiempo');

subplot(3,1,2);
plot(tiempo, velocidad, 'LineWidth', 1.3);
hold on;
plot(tiempo, velocidad_filtrada, 'LineWidth', 1.3);
xlabel('Tiempo [s]', 'FontName', 'Times New Roman', 'FontSize', 14);
ylabel('Velocidad [RPM]', 'FontName', 'Times New Roman', 'FontSize', 14);
legend({'Sin filtrar','Filtrada'}, 'Location','best');
grid on; title('Velocidad vs Tiempo');

subplot(3,1,3);
plot(tiempo, voltaje, 'LineWidth', 1.3);
xlabel('Tiempo [s]', 'FontName', 'Times New Roman', 'FontSize', 14);
ylabel('Voltaje [V]', 'FontName', 'Times New Roman', 'FontSize', 14);
grid on; title('Voltaje aplicado vs Tiempo');

sgtitle('Datos recibidos desde Arduino 6V input', 'FontName', 'Times New Roman', 'FontSize', 16);

%% === GUARDADO DE DATOS ======================================
filename = "datos_motor6V.csv";
T = table(tiempo, posicion, velocidad, velocidad_filtrada, voltaje, ...
    'VariableNames', {'tiempo_s','posicion_rev','velocidad_RPM','velocidad_filtrada_RPM','voltaje_V'});

writetable(T, filename);
fprintf('Datos guardados en "%s"\n', filename);
