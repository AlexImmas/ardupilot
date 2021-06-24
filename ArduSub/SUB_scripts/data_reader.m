%% Reading in File

filename = 'log.txt';

A = importdata(['C:\cygwin64\home\taflab\embedded_script\dronekit-python\scripts\SUB_scripts\' filename]);

%% Plotting 
figure()

subplot(2,1,1);
x = linspace(0,10);
y1 = sin(x);
plot(A(:,2),A(:,3))
xlabel("Longitude GPS (deg)")
ylabel("Latitude GPS (deg)")

subplot(2,1,2); 
y2 = sin(5*x);
plot(A(:,1),A(:,4))
xlabel("Time (sec)")
ylabel("Depth (m)")
