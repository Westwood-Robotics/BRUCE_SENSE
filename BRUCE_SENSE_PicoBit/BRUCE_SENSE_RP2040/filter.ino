// Calculate moving average
float moving_average(float *q, 
                     float new_data){
  q[Q_LEN]-=q[0]/Q_LEN; // Remove the portion of the first value in the queue from average
  for(int i=0; i<(Q_LEN-1); i++){
    q[i]=q[i+1];    
  } // Shift queue forward 
  q[Q_LEN-1] = new_data; // Add new data into queue
  q[Q_LEN]+= q[Q_LEN-1]/Q_LEN; // Add the portion of the last value in the queue into average
  return q[Q_LEN];
}
