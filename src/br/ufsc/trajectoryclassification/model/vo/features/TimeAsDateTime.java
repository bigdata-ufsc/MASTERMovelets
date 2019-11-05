package br.ufsc.trajectoryclassification.model.vo.features;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;

public class TimeAsDateTime implements IFeature<TimeAsDateTime> {

	private Date value;
	
	private SimpleDateFormat formatter = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
	
	public TimeAsDateTime(String value) throws ParseException {
	
		this.value = formatter.parse(value);
		
	}
	
	public Date getValue() {
		
		return value;	
	}

	
	@Override
	public String toString() {
		// TODO Auto-generated method stub
		
		String dateFormat = "yyyy/MM/dd HH:mm:ss";
		SimpleDateFormat sdf = new SimpleDateFormat(dateFormat, new Locale("UTC"));
		
		return sdf.format(this.value);
	}
	
	public double normalizeDistance(double distance, double maxValue ){
		
		/* If maxValue was not defined
		 * */
		if (maxValue == -1)
			return distance;
				
		if (distance >= maxValue)
			return Double.MAX_VALUE;
				
		return distance / maxValue;
		
	}

	@Override
	public double getDistanceTo(TimeAsDateTime other, FeatureComparisonDesc featureComparisonDesc) {
		// TODO Auto-generated method stub
		if (featureComparisonDesc == null)
			return Math.abs(((double)(this.value.getTime()-other.getValue().getTime()))/1000);
		
		/* ------------------------------------------------------------------ */
			
		switch (featureComparisonDesc.getDistance().toLowerCase() ) {
		
		case "difference": return normalizeDistance(Math.abs((double)(this.value.getTime()-other.getValue().getTime()))/1000, 
				featureComparisonDesc.getMaxValue());			
		
		default:
			break;
		}
		
		// TODO Auto-generated method stub
		return -1;
	}
	
	
	
	
}