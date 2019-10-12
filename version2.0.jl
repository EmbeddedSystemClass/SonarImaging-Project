println("Hello World")
using SerialPorts
using PyPlot 
using FFTW
sp = SerialPort("COM3:", 9600) # Opening serial port



#z[10000]


z=""
l = "" #replace z
d =""
m= "" #replace d

PyPlot.show()
while true
    println("Emptying buffer")
    h = readavailable(sp) 
    println("Number of bytes emptied from buffer ",h)
    
    println("Initiating chirp pulse")
    println("Sending c command to teensy")
    write(sp, "c") #writing to serial 
    sleep(0.2)
    
    s = readavailable(sp)
    if(s != "0")
        println("Unexpected response from teensy: Restart code")
        println("The value of unexpected s is ",s)
    end
    
    println("Received from c ", s)
    sleep(1) #delay
    
    println("Reading from the buffer by sending p command to teensy")
    write(sp, "p") #writing to serial
    global count=0
    global n=0
    global z= 0
    
while true
    
	x=readavailable(sp)
	if (length(x) == 0)
	   sleep(0.0001)
	   global n = n+1
	else
	   global z = string(z, x)#append to a string
	   global n = 0
	   global count = count+1
	end

    if n>1000
	  break
    end
       
end

println("count = ",count)
println("Number of bytes received from the teensy ",length(z))
Z = split(z, "\r\n")

#println(Z)
len = length(Z)-1;
v = Vector{Int64}(undef,len)

for n=1:len
	v[n] = parse(Int64, Z[n])
end
println("Number of ADC samples ",length(v))


#sleep(2)
#########################################################################################################################################
#########################################################################################################################################
########################################################################################################################################
#start another buffer Reading

println("Emptying buffer for the 2nd time")
h1 = readavailable(sp) 
println("Number of bytes emptied from buffer on the 2nd time ",h1)

println("Initiating chirp pulse for the second Receiver")
println("Sending s command to teensy to transmit chirp pulse")
write(sp, "s") #writing to serial 
sleep(0.2)

s = readavailable(sp)
if(s != "0")
	println("Unexpected response from teensy: Restart code")
	println("The value of unexpected s is ",s)
end

println("Received from s ", s)
sleep(1) #delay

println("Reading from the buffer by sending z command to teensy")
write(sp, "z") #writing to serial
global count1=0
global n1=0
global l =0

while true
    
	x1=readavailable(sp)
	if (length(x1) == 0)
	   sleep(0.0001)
	   global n1 = n1+1
	else
	   global l = string(l, x1) #append to array
	   global n1 = 0
	   global count1 = count1+1
	end

    if n1>1000
	  break
    end
       
end


println("count = ",count1)
println("Number of bytes received from the teensy ",length(l))
L = split(l, "\r\n") # making an array of values

len1 = length(L)-1;
e = Vector{Int64}(undef,len1)
for k=1:len1
     e[k] = parse(Int64, L[k])
end
println("Number of ADC samples from the second receiver is ",length(e))

##end of 2nd buffer reading
########################################################################################################################################
########################################################################################################################################

#Chirp pulse 2
c = 343; # Speed of sound in air in m/s
fs = 500000; # This is the sample rate of the sonar.
dt = 1/fs; # This is the sample spacing
r_max = 0.5; # Maximum range in metres to which to simulate.
t_max = 50E-3; # Time delay to max range
N = 32000 #ADC Buffer size
pulse_T = 6E-3; 
ds = 500000;
dts = 1/ds;
t_max_pulse = pulse_T;
t_max1=(20/343) +10E-3  
t_match=collect(0:dts:t_max1);
f=40000	

t_d=pulse_T/2;

t = (0:N-1).*dt; # t=0:dt:t_max defines a “range”.
print(t_max)
r = (c*t)/2;
f0 = 40000; # Centre frequency is 10 kHz
B = 4000; # Chirp bandwidth
T = 4E-3; # Chirp pulse length
K = B/T; # Chirp rate
rect(t) = (abs.(t) .<= 0.5)*1.0
v_tx = cos.( 2*pi*(f0*(t.-0.002 ) + 0.5*K*(t.-0.002 ).^2) ) .* rect.((t.-0.002)/T)

v_tx_match = cos.(2*pi*(f*(t_match.-t_d).+0.5*K*(t_match.-t_d).^2)).*rect((t_match .-t_d)/T);

r1=(343 .*t_match)/2
println("The length of v_tx_match is ", length(v_tx_match))
############################################################################################################################################
#Zero padding the cirp pulse to make it same size as the received signal
println()

len3 = length(v)
v_tx1=v_tx[1:len3]
len2=length(r1)-length(v)
j=zeros(len2)
append!(v,j)
len4=length(v)-length(v_tx_match)
###########################################################################################################################################
#Signal Processing for the first Receiver
V_TX1=fft(v_tx_match);
V_RX = fft(v)
H1 = conj(V_TX1);
V_MF1 = H1.*V_RX; # matched filter
v_mf = ifft(V_MF1);
v_mf = real(v_mf);

########################################################################################################################################
########################################################################################################################################

#Making an analytic signal for receiver 1
V_ANAL1 = 2*V_MF1;
N_ANAL1 = length(V_MF1);
println("The length of the matched filter array is: ", N_ANAL1);
if mod(N_ANAL1, 2)==0 # Case N even
	neg_freq_range = Int(N_ANAL1/2):N_ANAL1; # define the range of the neg_freq_range

else # Case N Odd
	neg_freq_range = Int((N_ANAL1+1)/2):N_ANAL1;
end

V_ANAL1[neg_freq_range].=0;
v_anal1 = ifft(V_ANAL1);

##########################################################################################################################################
#SIgnal processig for the second Receiver
println()
println("Processing data received from the second Receiver")

len5 = length(e)
v_tx2 = v_tx[1:len5]
len6 = length(r1)-length(e)
p =zeros(len6) # values used to zeropad the array
append!(e,j)
println("Done Zoro padding the received array")
len7=length(e)-length(v_tx_match)

#Signal Processing from the second receiver
println("Signal Processing from the second Receiver")
V_TX2 = fft(v_tx_match);
V_RX2 = fft(e)
H2 = conj(V_TX2);
V_MF2 = H2.*V_RX2;
v_mf2 = ifft(V_MF2);
v_mf2 = real(v_mf2);




#analytic Signal for the second receiver
V_ANAL2 = 2*V_MF2;
N_ANAL2 = length(V_MF2);
println("The length of the matched filter array is: ", N_ANAL2);
if mod(N_ANAL2, 2)==0 # Case N even
	neg_freq_range2 = Int(N_ANAL2/2):N_ANAL2; # define the range of the neg_freq_range

else # Case N Odd
	neg_freq_range2 = Int((N_ANAL2+1)/2):N_ANAL2;
end

V_ANAL2[neg_freq_range2].=0;
v_anal2 = ifft(V_ANAL2);

###############################################################################################################################################
#Applying windowing
t_rc = 50E-3;
dt_sample = 7.246e-6;
t_sample = collect(0:dt_sample:T);
t_rx = collect(0:dt_sample:t_rc);
Nw = length(t_rx);
df = 1/(Nw*dt_sample);
f_axis = (0:Nw-1)*df;
window = rect((f_axis)/B)
#Basebanding the signal

j=im;
v_bb1 = v_anal1.*exp.(-j*2*pi*f0*t_match); #Baseband signal for Analytic signal 1
v_bb2 = v_anal2.*exp.(-j*2*pi*f0*t_match); #Baseband signal for Analytic signal 2

V_BB1 = fft(v_bb1);
V_BB2 = fft(v_bb2);

phase_arr = angle.(v_bb1.*conj(v_bb2)) #calculating the phase array
##############################################################################################################################################
#making plots
s_m =4


PyPlot.clf()
subplot(s_m,1,1)
PyPlot.plot(r1,abs.(v_anal2))
title("Second Receiver Analytic Signal")
xlabel("Range in meters");
PyPlot.draw()
ylim([0,2E6])
xlim([0,3]);
subplot(s_m,1,2)
plot(r1,angle.(v_bb2))
title("Second Receiver Basband angle")
PyPlot.draw()
xlim([0,3]);
subplot(s_m,1,3)
plot(r1,angle.(v_bb1))
title("First Receiver Baseband angle")
PyPlot.draw()
xlim([0,3]);
subplot(s_m,1,4)
title("Wrapped Phase difference")
plot(r1, phase_arr,".")
xlim([0,3]);
PyPlot.draw()
PyPlot.sleep(0.05)



##############################################################################################################################################
#Some distance calculations


d = 0.018
k = 0;
lambda = 8.46e-3;
val_1 = phase_arr .+ k*2*pi
val_2 = lambda.*val_1
val_3 = 2*pi*d
val_4 = val_2./val_3
theta = asin.(val_4)
theta = theta.*(180/pi)

# println("The angle of arrival is ", theta)

x = r1.*cos.(theta)
y = r1.*sin.(theta)

#plot(x,y,".")




end


close(sp)