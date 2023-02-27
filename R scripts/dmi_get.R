library(tidyverse);library(httr);library(jsonlite);library(circular);library(lubridate)
#Get DMI data from metObs v1 REST API

# https://confluence.govcloud.dk/display/FDAPI
#API key: GET_YOUR_OWN_API_FROM_URL_ABOVE
#Weather station: "Flakkebjerg", stationId=06135
#Parameters: parameterId=wind_dir, parameterId=wind_speed
#From 2013-01-01: from=1356998400000000

# Time encoding should be like this 
# time_from <- "2019-05-01T00:00:00Z" #May 2019
# time_to <- "2021-10-01T00:00:00Z" #Oct 2021

get_kommune_data <- function(time_from , time_to , municipalityID = "0270", par = "mean_pressure", time_res = "hour"){
  climate_key <- GET_YOUR_OWN_API_FROM_URL_ABOVE
  stat <- "06135"
  par_dir <- "mean_wind_dir"
  par_wnd <- "mean_wind_speed"
  par_pres <- "mean_pressure"
  limit <- "300000"
  ##municipalityID Gribskov = 0270 https://danmarksadresser.dk/adressedata/kodelister/kommunekodeliste
  base <- paste0("https://dmigw.govcloud.dk/v2/climateData/collections/municipalityValue/items?limit=", limit)  
  
  request <- paste0(base,"&parameterId=", par, "&datetime=",time_from,"/",time_to,
                    "&municipalityId=",municipalityID,"&timeResolution=",time_res,"&api-key=", climate_key)
  response <- GET(request);response
  json <- content(response, as = "text") 
  df <- fromJSON(json);df
  df_clean <- df$features %>% 
    flatten() %>% 
    tibble() %>% 
    select(start_time = properties.from, variable = properties.parameterId, value = properties.value, time_res = properties.timeResolution) %>% 
    mutate(start_time = ymd_hms(start_time)+(60*60*2), value= value) %>% 
    select(nearest_hour = start_time,value)
  
  return(df_clean)
}

  
get_observation_data <- function(time_from , time_to , stationID = "06174", par = "precip_past1h"){
    climate_key <- GET_YOUR_OWN_API_FROM_URL_ABOVE
    par_dir <- "mean_wind_dir"
    par_wnd <- "mean_wind_speed"
    par_pres <- "mean_pressure"
    par_precip <- "precip_past10min"
    limit <- "300000"
    stationID <- "06174" ## Gribskov = 0270, Elbæk = 06174
    base <- paste0("https://dmigw.govcloud.dk/v2/metObs/collections/observation/items?limit=", limit)  
    
    request <- paste0(base,"&parameterId=", par, "&datetime=",time_from,"/",time_to,
                      "&stationId=",stationID,"&api-key=", climate_key)
    response <- GET(request);response
    json <- content(response, as = "text") 
    df <- fromJSON(json);df
    df_clean <- df$features %>% 
      flatten() %>% 
      tibble() %>% 
      select(start_time = properties.observed, variable = properties.parameterId, value = properties.value) %>% 
      mutate(start_time = ymd_hms(start_time)+(60*60*2), value= value) %>% 
      select(nearest_hour = start_time,value)
    
    return(df_clean)
  }
