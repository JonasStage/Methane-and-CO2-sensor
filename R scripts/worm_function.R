#### Worms function 
library(tidyverse);library(lubridate)

worm_cal <- read_csv("/Users/jonas/OneDrive - University of Copenhagen/Biologi/Methane_sensor/Kalibrering/worm_calibration_variables.csv") %>% 
  pivot_wider(names_from = var, values_from = value)

worm_function <- function(file_path, worm_number,air_sensor, rh = 100) {
source("/Users/jonas/OneDrive - University of Copenhagen/Biologi/Methane_sensor/Functions_CH4.R")

read_csv(file_path) %>% 
  mutate(datetime = ymd_hms(datetime), worm = worm_number, air_sensor = air_sensor) %>% 
  filter(SampleNumber > 100) -> worm

worm_kort <- worm %>% 
  select(datetime,CH4worm_kort, pt1000_kort, worm,PumpCycle, air_sensor, contains("dag"),tempC, SampleNumber) %>% 
  mutate(length = "kort") %>% 
  left_join(worm_cal, by = c("worm","length")) %>% 
  filter(between(CH4worm_kort, 0,2000)) %>% 
  mutate(abs_H = (6.112*exp((17.67*pt1000_kort)/(pt1000_kort+243.5))*rh*18.02)/((273.15+pt1000_kort)*rh*0.08314),
         V0 = abs_H*g+S,
         RsR0 = ((5000/CH4worm_kort)-1)/((5000/V0)-1),
         pred_CH4 = a*(RsR0^b)+c*abs_H*(a*RsR0^b) + K) %>% 
  filter(between(pred_CH4, -50, 50)) 


worm_lang <- worm %>% 
  select(datetime,CH4worm_lang, pt1000_lang, worm,PumpCycle, air_sensor, contains("dag"),tempC, SampleNumber) %>% 
  mutate(length = "lang") %>% 
  left_join(worm_cal, by = c("worm","length")) %>% 
  filter(between(CH4worm_lang, 1000,3000)) %>% 
  mutate(abs_H = (6.112*exp((17.67*pt1000_lang)/(pt1000_lang+243.5))*rh*18.02)/((273.15+pt1000_lang)*rh*0.08314),
         V0 = abs_H*g+S,
         RsR0 = ((5000/CH4worm_lang)-1)/((5000/V0)-1),
         pred_CH4 = a*(RsR0^b)+c*abs_H*(a*RsR0^b) + K) %>% 
  filter(between(pred_CH4, -50, 50))

air <- tibble(path = file_path, sensor = air_sensor) %>% 
  read_CH4_files(path, T) %>% 
  mutate(length = "air", air_sensor = air_sensor)

bind_rows(worm_kort,worm_lang, air) %>% 
  select(datetime:length, pred_CH4,K33_CO2,SampleNumber, PumpCycle,air_sensor, contains("dag"),tempC) %>% 
  mutate(PumpCycle = as.numeric(PumpCycle)) -> all_data

return(all_data)
}
