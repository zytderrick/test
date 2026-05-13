function main
  Test = 1;
  
  Path = './input_csv/';
  File = dir(fullfile(Path, '*.csv'));
  FileNames = {File.name}';
  
  OutPath = './output_file/';

  steer_files= "";
  speed_files = "";
  mpc_files = "";
  steer_index = 1;
  speed_index =1;
  mpc_index =1;
  for i=1:1:length(FileNames)
      if contains(FileNames(i), "steer")
          steer_files(steer_index) = FileNames{i};
          steer_index = steer_index+1;
      elseif contains(FileNames(i), "speed")
          speed_files(speed_index) = FileNames{i};
          speed_index = speed_index+1;
      elseif contains(FileNames(i), "mpc")
          mpc_files(mpc_index) = FileNames{i};
          mpc_index = mpc_index+1;
      end
  end

  if Test == 0
    if length(steer_files) == length(speed_files)
        for i=1:1:length(steer_files)
            Analysis_Lqr(Path +steer_files(i));
            Analysis_pid(Path +speed_files(i));
            name = OutPath + steer_files(i) + '.jpg';
            set(1, 'position', [0 0 1280 1440]);
            saveas(1,name);
        end
    end
  else
      for i=1:1:length(mpc_files)
        Analysis_mpc(Path + mpc_files(i));
        name = OutPath + mpc_files(i) + '.jpg';
        set(2, 'position', [0 0 1280 1440]);
        saveas(2,name);
      end
  end

end


function Analysis_Lqr(name)
  lqrdata = csvread(name, 2, 0);
  time = lqrdata(:,2);
  real_x = lqrdata(:,3);
  real_y = lqrdata(:,4);
  expect_x = lqrdata(:,24);
  expect_y = lqrdata(:,25);
  lateral = lqrdata(:,22);
  heading = lqrdata(:,23);
  figure(1);
  clf;
  % plot trajectory and expected
  subplot(4,2,1);
  subplot(4,2,[1 3]);
  hold on;
  grid on;
  plot(real_x, real_y, 'b.', 'Markersize',4);
  plot(expect_x, expect_y, 'r.','Markersize', 4);
  legend('real pos', 'target pos');
  xlabel('X [m]');
  ylabel('Y [m]');
  % plot lateral error
  subplot(4,2,5);
  hold on;
  grid on;
  yyaxis left;
  time = time - time(1);
  plot(time, lateral, 'b.', 'Markersize', 4);
  xlabel('time [s]');
  ylabel('Lateral Error [m]');
  yyaxis right;
  plot(time, heading, 'r.', 'Markersize', 4);
  legend('Lat', 'Head');
  xlabel('time [s]');
  ylabel('Heading Error [m]');
  % calc 
  lateral_threshold = [0.05, 0.1, 0.2, 0.3];
  lat_mean = mean(lateral);
  lat_rms = rms(lateral);
  lat_min = min(lateral);
  lat_max = max(lateral);
  lat_ratio = AnalysisArray(lateral, lateral_threshold);
  heading_threshold = [0.05, 0.1, 0.2, 0.3];
  head_mean = mean(heading);
  head_rms = rms(heading);
  head_max = max(heading);
  head_min = min(heading);
  head_ratio = AnalysisArray(heading, heading_threshold);
  
  subplot(4,2, 7);
  axis off;
  index = 1;
  str(index) = "Lat error detail";
  index = index +1;
  str(index) = "lat min: " + num2str(lat_min) + "m";
  index = index +1; 
  str(index) = "lat max: " + num2str(lat_max) + "m";
  index = index +1;
  str(index) = "lat mean: " + num2str(lat_mean) + "m";
  index = index +1;
  str(index) = "lat rms: " +num2str(lat_rms) + "m";
  for i=1:1:length(lat_ratio)
      str(index + i) = num2str(lat_ratio(i)*100) + "% < " + num2str(lateral_threshold(i)) + "m";
  end
  text(0.1, 0.5, str);
  
  index = 1;
  str(index) = "Head error detail";
  index = index +1;
  str(index) = "head min: " + num2str(head_min) + "rad";
  index = index +1;
  str(index) = "head max: " + num2str(head_max) + "rad";
  index = index +1;
  str(index) = "head mean: " + num2str(head_mean) + "rad";
  index = index +1;
  str(index) = "head rms: " + num2str(head_rms) + "rad";;
  for i=1:1:length(head_ratio)
      str(index + i) = num2str(head_ratio(i)*100) + "% < " + num2str(heading_threshold(i)) + "rad";
  end
  text(0.5, 0.5, str);
end

% 计算一维数组中小于thresh[一维数组]的output[比率]；
function output = AnalysisArray(array, thresh)
  for i = 1:1:length(thresh)
      output(i) = 0;
  end
  for i=1:1:length(array)
      for j=1:1:length(thresh)
          if abs(array(i)) < thresh(j)
              output(j) = output(j) +1;
          end
      end
  end
  for i = 1:1:length(output)
      output(i) = output(i) /length(array);
  end
end

function Analysis_pid(name)
  piddata = csvread(name, 2, 0);
  time = piddata(:,1);
  vel = piddata(:,4);
  target = piddata(:,7);
  error = piddata(:,10);
  time = time-time(1);
  % plot vel and target
  figure(1);
  hold on;
  subplot(4,2, [2 4]);
  hold on;
  grid on;
  plot(time, vel, 'b.','Markersize',4);
  plot(time, target, 'r.', 'Markersize', 4);
  legend('real vel', 'target vel');
  xlabel('Time [s]');
  ylabel('Velocity [m/s]');
  % plot error
  subplot(4,2,6);
  hold on;
  grid on;
  plot(time, error,'b.','Markersize',4);
  xlabel('Time [s]');
  ylabel('Vel Error [m/s]');
  % calc 
  max_target = max(target);
  vel_threshold = [0.05, 0.1, 0.15, 0.2];
  tmp_threshold = vel_threshold;
  for i=1:1:length(tmp_threshold)
      tmp_threshold(i) = tmp_threshold(i) * max_target; 
  end
  vel_mean = mean(error);
  vel_rms = rms(error);
  vel_min = min(error);
  vel_max = max(error);
  vel_ratio = AnalysisArray(error, tmp_threshold);
  
  subplot(4, 2, 8);
  axis off;
  index = 1;
  str(index) = "Vel error detail";
  index = index +1;
  str(index) = "vel min: " + num2str(vel_min) + "m/s";
  index = index +1; 
  str(index) = "vel max: " + num2str(vel_max) + "m/s";
  index = index +1;
  str(index) = "vel mean: " + num2str(vel_mean) + "m/s";
  index = index +1;
  str(index) = "vel rms: " +num2str(vel_rms) + "m/s";
  for i=1:1:length(vel_ratio)
      str(index + i) = num2str(vel_ratio(i)*100) + "% < " + num2str(vel_threshold(i) * 100) + "%";
  end
  text(0.1, 0.5, str);
end

function Analysis_mpc(name)
  mpcdata = csvread(name, 2, 0);
  time = mpcdata(:,1);
  real_x = mpcdata(:,2);
  real_y = mpcdata(:,3);
  lateral = mpcdata(:,31);
  heading = mpcdata(:,32);
  expect_x = mpcdata(:,33);
  expect_y = mpcdata(:,34);
  vel = mpcdata(:,5);
  target_vel = mpcdata(:,18);
  vel_error = mpcdata(:,19);
  
  figure(2);
  clf;
  hold on;
  % plot trajectory and expected
  subplot(4,2,1);
  subplot(4,2,[1 3]);
  hold on;
  grid on;
  plot(real_x, real_y, 'b.', 'Markersize',4);
  plot(expect_x, expect_y, 'r.','Markersize', 4);
  legend('real pos', 'target pos');
  xlabel('X [m]');
  ylabel('Y [m]');
  % plot lateral error
  subplot(4,2,5);
  hold on;
  grid on;
  yyaxis left;
  time = time - time(1);
  plot(time, lateral, 'b.', 'Markersize', 4);
  xlabel('time [s]');
  ylabel('Lateral Error [m]');
  yyaxis right;
  plot(time, heading, 'r.', 'Markersize', 4);
  legend('Lat', 'Head');
  xlabel('time [s]');
  ylabel('Heading Error [m]');
  % calc 
  lateral_threshold = [0.05, 0.1, 0.2, 0.3];
  lat_mean = mean(lateral);
  lat_rms = rms(lateral);
  lat_min = min(lateral);
  lat_max = max(lateral);
  lat_ratio = AnalysisArray(lateral, lateral_threshold);
  heading_threshold = [0.05, 0.1, 0.2, 0.3];
  head_mean = mean(heading);
  head_rms = rms(heading);
  head_max = max(heading);
  head_min = min(heading);
  head_ratio = AnalysisArray(heading, heading_threshold);
  
  subplot(4,2, 7);
  axis off;
  index = 1;
  str(index) = "Lat error detail";
  index = index +1;
  str(index) = "lat min: " + num2str(lat_min) + "m";
  index = index +1; 
  str(index) = "lat max: " + num2str(lat_max) + "m";
  index = index +1;
  str(index) = "lat mean: " + num2str(lat_mean) + "m";
  index = index +1;
  str(index) = "lat rms: " +num2str(lat_rms) + "m";
  for i=1:1:length(lat_ratio)
      str(index + i) = num2str(lat_ratio(i)*100) + "% < " + num2str(lateral_threshold(i)) + "m";
  end
  text(0.1, 0.5, str);
  
  index = 1;
  str(index) = "Head error detail";
  index = index +1;
  str(index) = "head min: " + num2str(head_min) + "rad";
  index = index +1;
  str(index) = "head max: " + num2str(head_max) + "rad";
  index = index +1;
  str(index) = "head mean: " + num2str(head_mean) + "rad";
  index = index +1;
  str(index) = "head rms: " + num2str(head_rms) + "rad";;
  for i=1:1:length(head_ratio)
      str(index + i) = num2str(head_ratio(i)*100) + "% < " + num2str(heading_threshold(i)) + "rad";
  end
  text(0.5, 0.5, str);
  
  % plot speed error
  subplot(4,2, [2 4]);
  hold on;
  grid on;
  plot(time, vel, 'b.','Markersize',4);
  plot(time, target_vel, 'r.', 'Markersize', 4);
  legend('real vel', 'target vel');
  xlabel('Time [s]');
  ylabel('Velocity [m/s]');
  % plot error
  subplot(4,2,6);
  hold on;
  grid on;
  plot(time, vel_error,'b.','Markersize',4);
  xlabel('Time [s]');
  ylabel('Vel Error [m/s]');
  % calc 
  max_target = max(target_vel);
  vel_threshold = [0.05, 0.1, 0.15, 0.2];
  tmp_threshold = vel_threshold;
  for i=1:1:length(tmp_threshold)
      tmp_threshold(i) = tmp_threshold(i) * max_target; 
  end
  vel_mean = mean(vel_error);
  vel_rms = rms(vel_error);
  vel_min = min(vel_error);
  vel_max = max(vel_error);
  vel_ratio = AnalysisArray(vel_error, tmp_threshold);
  
  subplot(4, 2, 8);
  axis off;
  index = 1;
  str(index) = "Vel error detail";
  index = index +1;
  str(index) = "vel min: " + num2str(vel_min) + "m/s";
  index = index +1; 
  str(index) = "vel max: " + num2str(vel_max) + "m/s";
  index = index +1;
  str(index) = "vel mean: " + num2str(vel_mean) + "m/s";
  index = index +1;
  str(index) = "vel rms: " +num2str(vel_rms) + "m/s";
  for i=1:1:length(vel_ratio)
      str(index + i) = num2str(vel_ratio(i)*100) + "% < " + num2str(vel_threshold(i) * 100) + "%";
  end
  text(0.1, 0.5, str);
end
