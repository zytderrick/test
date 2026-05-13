# Control Toolset

## 1. This aim of scripts is control analysis.
The scripts should run in MATLAB

### Input
  * The csv file of unprocessed data should be added to path
  * 

### Output
  * The output includes evaluation table and data figure

### Features
  * analysis_data.py: evaluate localization result, the input file is recorded localization points
  * analysis_v2.py: evaluate control result, include tracking performace, commands ..., the input file is recorded result file;
  * control_csv_analysis.m: evaluate control result, include tracking performace, commands ..., the input file is recorded result file;

## 2. How to run MATLAB scripts?

### 2.1 control_csv_analysis.m
```
step 1: add vehicle_state*.csv and trajectorylog*.csv to path
step 2: be aware if you need change the timestamp in the filename
step 3: run
```
### 2.2 lateral_tuning_data.m
```
step 1: add lateral_tuning_data.csv to path
step 2: run
```
### 2.3 lateral_tuning_log_data.m
```
step 1: add lateral_tuning_data.csv to path
step 2: run
```
## 3. How to run Python scripts?
### 3.1 analysis_v2.py
```
step 1: make sure there is folder named "/data" and "/result"
step 2: make sure there is vehicle_state*.csv in the folder "/data"
step 3: run in the terminal
```
### 3.2 analysis_data.py
```
step 1: make sure there is folder named "/data" and "/result"
step 2: make sure there is later_tuning_log_data.csv or later_tuning_data.csv in the folder "/data"
step 3: run in the terminal

