
function arduino_vals(hand_length, ROM, Vel, Reps)
%% Hand data from Kinovea 
close all        
x = [79.904167
    80.96250033
    79.63958367
    79.904167
    79.904167
    79.904167
    80.16875033
    80.16875033
    80.16875033
    80.16875033
    80.16875033
    80.16875033
    80.16875033
    79.904167
    79.904167
    79.904167
    79.904167
    80.16875033
    80.16875033
    80.43333367
    80.43333367
    80.697917
    80.96250033
    81.22708367
    81.22708367
    81.491667
    81.75625033
    82.02083367
    82.285417
    82.55000033
    82.81458367
    83.079167
    83.079167
    83.079167
    83.079167
    83.079167
    82.81458367
    82.81458367
    82.55000033
    82.285417
    82.285417
    81.75625033
    81.75625033
    81.491667
    81.491667
    81.491667
    81.491667
    81.491667
    81.22708367
    81.22708367
    80.697917
    80.43333367
    80.16875033
    79.63958367
    78.84583367
    78.05208367
    77.78750033
    77.25833367
    76.729167
    76.20000033
    75.67083367
    74.87708367
    73.81875033
    73.02500033
    71.43750033
    70.379167
    69.05625033
    67.73333367
    66.67500034
    65.88125034
    64.55833367
    62.70625034
    60.854167
    59.266667
    56.62083367
    53.97500034
    52.122917
    50.535417
    49.21250034
    48.41875034
    47.62500034
    40.21666701
    38.89375034
    37.57083367
    36.24791701
    34.66041701
    33.07291701
    31.48541701
    30.16250034
    28.83958367
    27.78125034
    26.72291701
    25.66458367
    24.60625034
    23.28333367
    21.96041701
    20.90208367
    19.84375034
    18.78541701
    17.99166701
    17.19791701
    16.40416701
    15.87500034
    15.34583367
    14.81666701
    14.28750034
    13.75833368
    12.96458368
    12.70000034
    12.43541701
    12.17083368
    11.90625034
    11.11250034
    10.84791701
    7.937500342
    6.350000343
    4.233333676
    1.852083677
    0.264583677
    0.264583677
    0.264583677
    0.264583677
    0.264583677
    0.793750343
    0.793750343
    0.793750343
    0.793750343
    0.52916701
    0.264583677
    3.434E-07
    ];
                            y = [19.314583
    18.78541633
    17.99166633
    17.99166633
    17.46249967
    17.19791633
    17.19791633
    17.19791633
    17.19791633
    17.19791633
    17.19791633
    17.46249967
    17.99166633
    17.99166633
    17.99166633
    18.25624967
    18.78541633
    18.520833
    18.520833
    18.520833
    18.78541633
    18.78541633
    18.78541633
    18.78541633
    19.04999967
    19.04999967
    19.04999967
    19.04999967
    18.78541633
    19.04999967
    18.78541633
    18.520833
    18.25624967
    18.25624967
    17.99166633
    17.727083
    17.46249967
    17.46249967
    17.46249967
    17.46249967
    16.933333
    15.61041633
    14.552083
    14.28749967
    14.28749967
    14.02291633
    13.758333
    13.22916633
    12.43541633
    11.11249967
    9.524999668
    7.408333002
    5.556249668
    4.233333002
    2.645833002
    1.322916336
    0.264583002
    -0.529166998
    -1.852083664
    -3.175000331
    -4.233333664
    -5.55625033
    -6.879166997
    -8.202083663
    -9.52500033
    -10.58333366
    -11.641667
    -12.70000033
    -13.229167
    -13.75833366
    -14.28750033
    -14.816667
    -15.08125033
    -15.34583366
    -16.13958366
    -16.93333366
    -17.46250033
    -17.46250033
    -17.72708366
    -16.404167
    -16.66875033
    -19.05000033
    -19.05000033
    -19.05000033
    -18.785417
    -18.25625033
    -17.72708366
    -17.46250033
    -17.197917
    -16.66875033
    -16.13958366
    -15.87500033
    -15.34583366
    -14.816667
    -14.55208366
    -14.022917
    -13.49375033
    -12.96458366
    -12.70000033
    -12.17083366
    -11.641667
    -11.37708366
    -11.37708366
    -11.11250033
    -10.58333366
    -9.789583663
    -9.52500033
    -8.73125033
    -8.466666997
    -8.202083663
    -7.93750033
    -7.672916997
    -7.14375033
    -6.879166997
    -7.93750033
    -6.35000033
    -5.820833664
    -3.96875033
    -0.529166998
    -0.264583664
    -3.309E-07
    0.264583002
    0.529166336
    0.793749669
    1.058333002
    1.058333002
    1.058333002
    0.793749669
    0.264583002
    -3.309E-07
    ];
    %% Scale input x,y values for different hand legnths and fit fourth order polynomial 
    hand_length_scaled = hand_length/22;
    scaling_matrix = [hand_length_scaled 0 0; 0 hand_length_scaled 0; 0 0 1];
    for i = 1:length(x)
        mat = [x(i); y(i); 1];
        J = scaling_matrix * mat;
        x(i) = J(1); y(i) = J(2);
    end
    
    figure(1);
    plot(x,y, 'r')
    xlabel("x position");
    ylabel("y position");
    
    P = polyfit(x,y,4);
    hold on;
    plot(x, polyval(P, x), 'b');
    legend("Original Curve", "Fitted Curve")
    
    fprintf("equation = %f x^4 + %f x^3 + %f x^2 + %f x + %f\n", P(1), P(2), P(3), P(4), P(5))
    
    %% Generate desired trajectory from user input and fitted curve 
    [q,qd,qdd] = mjtTrajectoryGeneration(ROM,Vel,Reps,P);
    
    % Unpack desired position
    theta_val = q.data(1,1,:);
    r_val = q.data(2,1,:);
    theta_val = reshape(theta_val, 1, size(theta_val,3));
    r_val = reshape(r_val,1,size(r_val,3));
    
    % Uncomment for downsampled "copy and paste" trajectories 
    %{
    theta_val = downsample(theta_val, 12);
    r_val = downsample(r_val, 12);
    fprintf('theta_val:\n')
    fprintf('[')
    for i = 1:length(theta_val)-1
        fprintf(num2str(theta_val(i)))
        fprintf(", ")
    end
    fprintf(num2str(theta_val(length(theta_val))))
    fprintf(']\n')
    fprintf('r_val:\n')
    fprintf('[')
    for i = 1:length(r_val)-1
        fprintf(num2str(r_val(i)))
        fprintf(", ")
    end
    fprintf(num2str(r_val(length(r_val))))
    fprintf(']\n')
    %}
    
    % Convert theta to degrees and r to cm 
    theta_deg = theta_val .*180/pi;
    r_val = r_val.*0.1; 
    
    % Define trajectory for 1 rep create trajectory vectors  
    rep_theta = [theta_deg flip(theta_deg)];
    rep_r = [r_val flip(r_val)];
    theta_trajectory = [];
    r_trajectory = [];
    for i = 1:Reps
        theta_trajectory = [theta_trajectory rep_theta];
         r_trajectory = [r_trajectory rep_r];
    end
    time = linspace(0,q.time(end),size(theta_trajectory,2));
    
    figure(2)
    plot(time, theta_trajectory)
    title("Desired Revolute Position vs Time")
    xlabel("Time(s)")
    ylabel("Position (deg)")
    
    figure(3)
    plot(time, r_trajectory)
    title("Desired Prismatic Position vs Time")
    xlabel("Time(s)")
    ylabel("Position (mm)")
    
    % Save to txt file with formating [theta,r]
    y = [rep_theta;rep_r]; 
    fileID = fopen('trajectory.txt','w'); 
    % fprintf(fileID,'Desired Trajectory [theta,r]\n\n'); 
    fprintf(fileID,'%f %f\n',y); 
    fclose(fileID); 
    type trajectory.txt
    
end
