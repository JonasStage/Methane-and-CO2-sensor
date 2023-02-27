library(TTR);library(tidyverse);library(lubridate);library(ggpubr)

model_coef <- read_csv("/Users/jonas/OneDrive - University of Copenhagen/Biologi/Methane_sensor/Kalibrering/all_sensor_model_coef.csv") %>% 
  mutate(sensor = case_when(sensor_desc == "sensor1_grøn" ~ "1grøn",
                            sensor_desc == "sensor4_grøn" ~ "4grøn", 
                            T ~ sensor)) %>% 
  select(-...9)
areal = 0.0615;volumen = 0.0135
sem <- function(x) sd(x, na.rm=T)/sqrt(length(x))

tema <-   theme(panel.background = element_blank(),
                panel.grid.major = element_blank(),
                panel.grid.minor = element_blank(),
                strip.text = element_text(size = 16),
                legend.text = element_text(size = 22),
                legend.title = element_text(size = 22),
                plot.title = element_text(hjust = 0.5), 
                axis.text = element_text(size = 16),
                axis.title = element_text(size = 22),
                strip.background = element_rect(fill = "white"),
                panel.border = element_rect(colour = "black", fill=NA))

ppm_to_umol <- function(pressure, CH4_ppm, volume, temperature_C, area) {
  ((pressure*CH4_ppm*volume)/(8.314*(temperature_C+ 273.15)))/area
}

read_CH4_files <- function(data, files, pump_present = T) {
  data <- data %>% 
    inner_join(model_coef, by ="sensor") %>% rename(files = path)
  data %>% 
    mutate(data = lapply(files,read_csv, show_col_types = T, col_types = list(
                            col_double(),col_double(),col_character(),col_double(),
                            col_double(),col_double(),col_double(),col_double(),
                            col_double(),col_double(),col_double(),col_double(),col_double()))) %>% 
    unnest(data) %>%
    mutate(datetime = ymd_hms(datetime),
           abs_H = (6.112*exp((17.67*tempC)/(tempC+243.5))*`RH%`*18.02)/((273.15+tempC)*100*0.08314),
           V0 = abs_H*g+S,
           RsR0 = ((5000/CH4smV)-1)/((5000/V0)-1),
           pred_CH4 = a*(RsR0^b)+c*abs_H*(a*RsR0^b) + K)  %>% 
    select(files, datetime, `RH%`:tempC,K33_RH:ncol(.), pred_CH4, sensor, abs_H, contains("volumen"), contains("station")) -> done_data
  return(done_data) }

baggrund_ppm <- 1.8916

running_5mean_function <- function(data,pred_CH4 = pred_CH4) {data %>% 
  group_by(PumpCycle,dag,sensor) %>% 
  add_tally() %>% 
  filter(n > 100) %>% 
  mutate(pred_CH4_mean = runMean(pred_CH4, 10),
         pred_CH4_mean = runMean(pred_CH4_mean, 10),
         pred_CH4_mean = runMean(pred_CH4_mean, 10),
         pred_CH4_mean = runMean(pred_CH4_mean, 10),
         pred_CH4_mean = runMean(pred_CH4_mean, 10))
  
  #umol_CH4_mean = runMean(umol_CH4, 10),
  #umol_CH4_mean = runMean(umol_CH4_mean, 10),
  #umol_CH4_mean = runMean(umol_CH4_mean, 10),
  #umol_CH4_mean = runMean(umol_CH4_mean, 10),
  #umol_CH4_mean = runMean(umol_CH4_mean, 10)
  }

ebullutive_flux <- function(data,station,IndexSpan = 30,runvar_cutoff = 0.005, show_plots = T, CH4_diffusion_cutoff = 1, number_of_pumpcycles_in_plot = 24) {
  par(ask=T)
  GetIDsBeforeAfter = function(x,IndexSpan) {
    v = (x-IndexSpan) : (x+IndexSpan)
    v[v > 0]
  }  
  
  data %>%  
    select(-contains("K33")) %>% 
    filter(n > 100) %>% 
    drop_na(pred_CH4_mean) %>% 
    mutate(run_var5 = runVar(pred_CH4_mean, n = 5)) %>% 
    ungroup() %>% 
    mutate(row = row_number()) %>%  
    {. ->> running_var} %>% 
    filter(run_var5 > runvar_cutoff) %>% 
    mutate(time_diff = datetime -lag(datetime),
           time_diff = as.numeric(time_diff)) %>%
    drop_na(time_diff) %>% 
    mutate(gruppering =  1 + cumsum(time_diff>6)) %>% 
    group_by(gruppering) %>% 
    pull(row) %>%                  
    map(~GetIDsBeforeAfter(., IndexSpan)) %>%  
    unlist() %>%                
    unique() -> ids_to_remain
  
  running_var %>% 
    group_by(station, PumpCycle) %>% 
    mutate(PumpCycle_Timediff = max(datetime)-min(datetime),
           PumpCycle_Timediff =as.numeric(PumpCycle_Timediff, units = "hours")) %>% 
    summarise(sum_bobler = 0,
              n_bobler = 0,
              PumpCycle_Timediff = mean(PumpCycle_Timediff), 
              PumpCycle_Timediff_hr = as.numeric(PumpCycle_Timediff),
              temp = mean(tempC,na.rm=T)) %>% 
    rename(time = PumpCycle_Timediff_hr) %>% 
    select(-PumpCycle_Timediff) ->no_bobler
  
  running_var %>% 
    filter(row %in% ids_to_remain) %>% 
    mutate(time_diff = datetime -lag(datetime),
           time_diff = as.numeric(time_diff)) %>% 
    drop_na(time_diff) %>% 
    group_by(station, PumpCycle) %>% 
    mutate(gruppering =  1 + cumsum(time_diff>6)) %>% 
    group_by(station,gruppering,PumpCycle) %>% 
    mutate(first = first(pred_CH4),
           last = last(pred_CH4),
           first = if_else(is.na(first), first(pred_CH4), first),
           last = if_else(is.na(last), last(pred_CH4), last)) %>% 
    {. ->> bubbles_check1} %>% 
    filter(first < last) %>% 
    bind_rows(filter(running_var, !row %in% ids_to_remain))  %>% 
    {. ->> bubbles_check2} %>% 
    arrange(row) %>% 
    mutate(PumpCycle_Timediff = as.numeric(max(datetime)-min(datetime), units = "hours")) %>% 
    summarize(time_diff = max(datetime)-min(datetime),
              min_datetime = datetime[which.min(pred_CH4)],
              max_datetime = datetime[which.max(pred_CH4)],
              datetime = mean(datetime),
              min_CH4 = min(pred_CH4, na.rm=T),
              max_CH4 = max(pred_CH4, na.rm=T),
              CH4_diff = max_CH4-min_CH4,
              PumpCycle_Timediff = mean(PumpCycle_Timediff),
              temp = mean(tempC, na.rm=T)) %>%
    ungroup() %>% 
    {. ->> bubbles_detected} %>% 
    filter(CH4_diff > CH4_diffusion_cutoff & min_datetime < max_datetime) %>% 
    drop_na(gruppering) %>% 
    add_count(station,PumpCycle) %>%
    {. ->> n_bubbles_per_pump} %>% 
    rename(sum_bobler = CH4_diff,
           time = PumpCycle_Timediff,
           n_bobler = n) %>% 
    mutate(index = IndexSpan) %>% 
    bind_rows(no_bobler) %>% 
    group_by(station,PumpCycle) %>% 
    summarise(sum_bubbles_concentration = sum(sum_bobler),
              n_bubbles = sum(n_bobler),
              pumpcycle_duration_hr = max(time),
              temp = mean(temp, na.rm=T),
              bubbles_per_time = n_bubbles/pumpcycle_duration_hr,
              concentration_per_time = sum_bubbles_concentration/pumpcycle_duration_hr) -> bubbles_found
  
  plotting_data <- running_var %>% 
    mutate(plot_number = floor(PumpCycle/number_of_pumpcycles_in_plot))
  
  if(show_plots) {for(i in unique(plotting_data$station)) {
    wp_select = i
    bubbles_found %>% filter(station == wp_select) %>% 
      ungroup %>% summarize(n_bubles = sum(n_bubbles)) -> bubles_count 
    cat("waypoint ",i," has a total of ",bubles_count$n_bubles, "bubbles\n")
    for(j in unique(filter(plotting_data, station == wp_select)$plot_number)){
      par(ask=T)
      plot_number_select = j;plotting_data %>% 
        filter(station == wp_select , plot_number == plot_number_select) ->plot1_dat
      plot1_dat %>% 
        ggplot(aes(datetime,pred_CH4_mean, group = PumpCycle)) +
        geom_point() + 
        geom_point(data = filter(drop_na(bubbles_check2, gruppering), station == wp_select), aes(datetime, pred_CH4_mean), col = "blue") +
        geom_vline(data = filter(n_bubbles_per_pump, station == wp_select), aes(xintercept= datetime), col = "red")+
        scale_x_datetime(limits=c(min(plot1_dat$datetime), max(plot1_dat$datetime))) + 
        scale_y_continuous(limits=c(min(plot1_dat$pred_CH4_mean,na.rm=T), max(plot1_dat$pred_CH4_mean,na.rm=T))) + 
        ggtitle(i) -> graf1
      ggplot() +
        geom_point(data = filter(plot1_dat, run_var5 > 0.1), aes(datetime, run_var5), col = "red") + 
        geom_point(data = filter(plot1_dat, run_var5 > 0.2), aes(datetime, run_var5), col ="blue") + 
        geom_point(data = filter(plot1_dat, run_var5 > 0.5), aes(datetime, run_var5), col ="green") + 
        ggtitle("Variance cutoff\nred = 0.1\nblue = 0.2\ngreen = 0.5") + 
        scale_x_datetime(limits=c(min(plot1_dat$datetime), max(plot1_dat$datetime))) + 
        scale_y_continuous(limits=c(0.1,max(plot1_dat$run_var5,na.rm=T))) +
        geom_hline(yintercept = plot1_dat$runvar_cutoff) + theme(legend.position=c(.9,.75))->graf2
      ggarrange(graf1,graf2, align = "h", ncol = 1) ->p
      print(p)
    }}} else {}
  par(ask=F) 
  return(bubbles_found)}


diffusive_flux <- function(data, station, runvar_cutoff = 0.005, show_plots = T,IndexSpan = 30, number_of_pumpcycles_in_plot = 50) {
  
  GetIDsBeforeAfter = function(x,IndexSpan) {
    v = (x-IndexSpan) : (x+IndexSpan)
    v[v > 0]
  }  
  
data %>% 
  select(-contains("K33")) %>% 
  filter(n > 100) %>% 
  drop_na(pred_CH4_mean) %>% 
  mutate(run_var5 = runVar(pred_CH4_mean, n = 5)) %>% 
  ungroup() %>% 
  mutate(row = row_number()) %>% 
  {. ->> running_var_diff} %>%  
  filter(run_var5 > runvar_cutoff) %>% 
  mutate(time_diff = datetime -lag(datetime),
         time_diff = as.numeric(time_diff)) %>%
  drop_na(time_diff) %>% 
  mutate(gruppering =  1 + cumsum(time_diff>6)) %>% 
  group_by(gruppering) %>% 
  pull(row) %>%                  
  map(~GetIDsBeforeAfter(., IndexSpan)) %>%  
  unlist() %>%                
  unique() -> ids_to_remain_diff

running_var_diff %>% 
  filter(!row %in% ids_to_remain_diff) %>% 
  mutate(time_diff = datetime -lag(datetime),
         time_diff = as.numeric(time_diff)) %>% 
  drop_na(time_diff) %>% 
  mutate(gruppering =  1 + cumsum(time_diff>6)) %>% 
  arrange(row) %>% 
  {. ->> bubbles_diff} %>% 
  group_by(PumpCycle,dag,station) %>% 
  mutate(baggrunds_konc = 1.8916,
         first = first(pred_CH4),
         first = if_else(is.na(first),pred_CH4,first)) %>%
  drop_na(time_diff) %>% 
  mutate(min_grp = min(gruppering)) %>% 
  filter(gruppering == min_grp) %>% 
  drop_na(pred_CH4_mean) %>% 
  filter(between(row_number(),200,600)) %>% 
  #slice_head(n = 500) %>% 
  {. ->> dif_check} %>% 
  mutate(time = datetime-min(datetime)) %>% 
  nest() %>% 
  mutate(model = map(data, ~lm(pred_CH4 ~ time, data = .)),
         slope = map(model, coef),
         n     = map(data, tally),
         r2    = map(model, summary),
         r2    = map_dbl(r2, "r.squared"),
         temp  = map_dbl(data, ~mean(.$tempC))) %>% 
  unnest_wider(slope) %>% 
  unnest_wider(n) %>%
  {. ->> model_check} %>% 
  mutate(slope_CH4_hr = time*3600,
         station = station) %>% 
  select(slope_CH4_hr,n, r2,temp,station)-> diffusive_flux


  plotting_data <- dif_check %>% 
  mutate(plot_number = floor(PumpCycle/number_of_pumpcycles_in_plot))
  data_indelt <- data %>% 
  mutate(plot_number = floor(PumpCycle/number_of_pumpcycles_in_plot))
  
if(show_plots){for(i in unique(plotting_data$station)) {
  wp_select = i
  for(j in unique(filter(plotting_data, station == wp_select)$plot_number)){
    plot_number_select = j;
    plotting_data %>% 
      filter(station == wp_select , plot_number == plot_number_select) ->plot_lm
    data_indelt %>% 
      filter(station == wp_select , plot_number == plot_number_select) ->plot_raw
  par(ask=T)
  plot_lm %>% 
  ggplot(aes(datetime, pred_CH4_mean, group = PumpCycle)) + 
  geom_point(data = plot_raw, aes(datetime, pred_CH4, group = PumpCycle, col = "Raw Data")) +
  geom_point(data = plot_lm, aes(datetime, pred_CH4_mean, group = PumpCycle)) +
  geom_smooth(method = "lm", se =F) + ggtitle(i) ->p
print(p);print(i)
}}} else {}
par(ask=F)

return(diffusive_flux) }
