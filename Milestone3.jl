println("Hello World")
using SerialPorts
using PyPlot 
using FFTW
sp = SerialPort("COM3:", 9600) # Opening serial port



#z[10000]


z=""

d =""
write(sp, "c") #writing to serial 
sleep(0.5) #delay
write(sp, "p") #writing to serial


while true
    
	x=readavailable(sp)
    #global X = Vector{UInt8}(x)
        
	if bytesavailable(sp)<1
		sleep(0.5)
		if bytesavailable(sp)<1
			break
		end
	end
	

	#println(x)
	
	global z = string(d, x)
	#X = Vector{UInt8}(x)
	#println(z)
        
end
Z = split(z, "\r\n")

#println(length(Z))
len = length(Z)-1
v = Vector{Int64}(undef,len)

for n=1:len
	v[n] = parse(Int64, Z[n])
end
#println(typeof(v))

#Chirp pulse 2
c = 343; # Speed of sound in air in m/s
fs = 38*44100; # This is the sample rate of the sonar.
dt = 1/fs; # This is the sample spacing
r_max = 0.5; # Maximum range in metres to which to simulate.
t_max = 50E-3; # Time delay to max range

t = collect(0:dt:t_max); # t=0:dt:t_max defines a “range”.
print(t_max)
# Create an array containing the range values of the samples
r = (c*t)/2;
f0 = 38400; # Centre frequency is 10 kHz
B = 4000; # Chirp bandwidth
T = 4E-3; # Chirp pulse length
K = B/T; # Chirp rate
rect(t) = (abs.(t) .<= 0.5)*1.0
v_tx = cos.( 2*pi*(f0*(t.-0.002 ) + 0.5*K*(t.-0.002 ).^2) ) .* rect.((t.-0.002)/T)

#received signal processing
v=(v/2047.5)

#len2= length(r)-length(v)
#b = zeros(len2)

#append!(v,b)
#println(v)
println()

len3 = length(v)
#c=zeros(len3)
v_tx1=v_tx[1:len3]
println(length(v))
println(length(v_tx1))
#append!(v_tx,c)
#println(v)

#Matched filter
V_TX = fft(v_tx1)

V_RX = fft(v)

H = conj(V_TX)

V_MF = H.*V_RX

v_mf = ifft(V_MF)
v_mfa = v_mf.+6
#plot(v_mfa)
#plot(v_mf)

v_mfr = real(v_mfa)
plot(v_mfr)

#plot(t,v_mfr)
#println(v_mf)

#Plotting
# PyPlot.clf()
# subplot(2,1,1)

# title("Matched filter output")
# PyPlot.draw()
# xlim([0,10])

#Analytic signal

# V_ANAL = 2*V_MF
# N = length(V_MF)
# if mod(N,2)==0 # case N even
# 	neg_freq_range = Int(N/2):N; # Define range of “neg-freq” components
# else # case N odd
# 	neg_freq_range = Int((N+1)/2):N;
# end

# V_ANAL[neg_freq_range] .= 0; # Zero out neg components in 2nd half of
# v_anal = ifft(V_ANAL);
# subplot(2,1,2)
# plot(r,abs.(v_anal))
# title("Analytical Signal")
# xlabel("Range in meters");
# PyPlot.draw()
# PyPlot.sleep(0.05)

close(sp)