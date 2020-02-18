% Record your voice for 5 seconds.
recObj = audiorecorder;
disp('Start speaking.')
recordblocking(recObj, 5);
disp('End of Recording.');

% Play back the recording.
play(recObj);

% Store data in double-precision array.
myRecording = getaudiodata(recObj);
figure; plot(myRecording); % Plot the original waveform.

% cut the area you want and convert it into integers
MyAudioArray = uint16((myRecording(2000:8000)+1)*1024/2);
csvwrite('AudioArray.csv', MyAudioArray');
% Plot the modified waveform.
figure;plot(MyAudioArray);


s = serialport("COM3", 115200);
%s.OutputBufferSize=4001; %8000 bits data size, needed because uart_receive takes uint8_t*
fopen(s);
AudioArray = readtable('AudioArray.csv');
fwrite(s, AudioArray, 'uint16', 'async');
pause(1.5);
fclose(s);
delete(s);
