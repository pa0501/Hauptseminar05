package parkingRobot.hsamr0.util;

import java.util.ArrayList;

public class Average {
	
	public ArrayList<Double> values;
	public int index_values = 0;
	
	public int num_values = 0;
	public int num_values_max = -1;		// -1 -> No maximum number
	
	public boolean resetAfterMax = false;
	
	public Average() {
		values = new ArrayList<Double>() ;
	}
	
	public Average(int num_max, boolean resetAfterMaxNum) {
		values = new ArrayList<Double>();
		
		num_values_max = num_max;
		resetAfterMax = resetAfterMaxNum;
	}
	
	public void addValue(double value) {	
		if (num_values < num_values_max) {
			num_values++;
		}
		
		if (values.size() >= num_values_max) {
			values.set(index_values, value);
		} else {
			values.add(index_values, value);
		}
		
		index_values++;
		
		if (index_values >= num_values_max - 1) {
			
			index_values = 0;
		}
		
		
	}
	
	public double getAverage() {
		double sum = 0;
		
		if (index_values == 0) {
			return 0;
		}
		
		for (int i=0; i<index_values; i++) {
			sum += values.get(i);
			
			System.out.print(values.get(i) + " + ");
		}
		
		double average = sum / index_values;
		
		return average;
	}
	
	public double getAverageInRange(int start, int end) {
		
		double sum = 0;
		int num_summed = 0;
		
		if (index_values == 0) {
			return 0;
		}

		int s = ((index_values - (num_values_max  - start)) + num_values_max) % num_values;
		int e = ((index_values - (num_values_max  - end) ) + num_values_max) % num_values;
		
		s = Math.max(0, s);
		
		if (e < s) {
			for (int i=s; i<num_values; i++) {
				sum += values.get(i);
				num_summed++;
			}
			
			for (int i=0; i<=e; i++) {
				sum += values.get(i);
				num_summed++;
			}
		} else {
			e = Math.min(e, values.size());
			
			for (int i=s; i<=e; i++) {
				sum += values.get(i);
				num_summed++;
			}
		}
		
		double average = sum / num_summed;
		
		return average;
	}
	
	public double getAverageOfLastN(int n) {
		return getAverageInRange(num_values - n, num_values - 1);
	}
	
	
	public void reset() {
		this.num_values = 0;
		this.index_values = 0;
	}
}
