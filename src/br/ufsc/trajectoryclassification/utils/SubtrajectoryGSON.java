package br.ufsc.trajectoryclassification.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.apache.commons.lang3.ArrayUtils;

import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQuality;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureQuality;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;

public class SubtrajectoryGSON {
	
	
	private int start;

	private int end;
 
	private int trajectory;
	
	private String label;

	private HashMap<String, IFeature> features;
	
	private List<HashMap<String, IFeature>> points_with_only_the_used_features;
		
	private HashMap<String, Double> maxValues;
	
	private int[] pointFeatures;
	
	private double[] splitpoints;
	
	private double[][] distances;
	
	private int[] positions;
	
	private List<IPoint> Data;
	
	private Map<String,Double> quality;

	public SubtrajectoryGSON(int start, int end, int trajectory, String label,
			HashMap<String, IFeature> features, int[] pointFeatures, double [] splitpoints, double[][] distances, ISubtrajectory[] bestAlignments, IQuality quality, Description description, List<IPoint> Data, List<HashMap<String, IFeature>> only_used_features) {
		super();
		this.start = start;
		this.end = end;
		this.trajectory = trajectory;
		this.label = label;
		this.features = features;		
		this.distances = distances;
		this.positions = Arrays.asList(bestAlignments).stream().mapToInt(e -> (e!=null) ? e.getStart() : -1).toArray();
		this.pointFeatures = pointFeatures;
		this.splitpoints = splitpoints;
		this.quality = quality.getData();
		this.maxValues = new HashMap<>();
		this.points_with_only_the_used_features = only_used_features;
		this.Data = Data;
		
		for (FeatureComparisonDesc featureComparisonDesc : description.getPointComparisonDesc().getFeatureComparisonDesc()) {
			maxValues.put(featureComparisonDesc.getText(), featureComparisonDesc.getMaxValue());				
		}

		for (FeatureComparisonDesc featureComparisonDesc : description.getSubtrajectoryComparisonDesc().getFeatureComparisonDesc()) {
			maxValues.put(featureComparisonDesc.getText(), featureComparisonDesc.getMaxValue());				
		}

	}
	
	public SubtrajectoryGSON(int start, int trajectory, String label, int[] pointFeatures, double [] splitpoints, double[][] distances, ISubtrajectory[] bestAlignments, IQuality quality, List<HashMap<String, IFeature>> only_used_features) {
		super();
		this.start = start;
		this.trajectory = trajectory;
		this.label = label;	
		this.pointFeatures = pointFeatures;
		this.splitpoints = splitpoints;
		this.distances = distances;
		this.positions = Arrays.asList(bestAlignments).stream().mapToInt(e -> (e!=null) ? e.getStart() : -1).toArray();
		this.quality = quality.getData();
		this.points_with_only_the_used_features = only_used_features;

	}

	public int getStart() {
		return start;
	}

	public void setStart(int start) {
		this.start = start;
	}

	public int getEnd() {
		return end;
	}

	public void setEnd(int end) {
		this.end = end;
	}

	public int getTrajectory() {
		return trajectory;
	}

	public void setTrajectory(int trajectory) {
		this.trajectory = trajectory;
	}

	public String getLabel() {
		return label;
	}

	public void setLabel(String label) {
		this.label = label;
	}

	public List<IPoint> getData() {
		return Data;
	}
	
	public void setUsedFeatures(List<HashMap<String, IFeature>> Data) {
		this.points_with_only_the_used_features = Data;
	}
	
	public void setData(List<IPoint> Data) {
		this.Data = Data;
	}
	
	public List<HashMap<String, IFeature>> getOnlyUsedFeatures() {
		return points_with_only_the_used_features;
	}
	
	public double[] getSplitpoints() {
		return splitpoints;
	}
	
	public void setSplitpoints(double[] splitpoints) {
		this.splitpoints = splitpoints;
	}

	public HashMap<String, IFeature> getFeatures() {
		return features;
	}

	public void setFeatures(HashMap<String, IFeature> features) {
		this.features = features;
	}

	public double[][] getDistances() {
		return distances;
	}

	public void setDistances(double[][] distances) {
		this.distances = distances;
	}

	public Map<String, Double> getQuality() {
		return quality;
	}
	
	public void setQuality(Map<String, Double> quality) {
		this.quality = quality;
	}
	
		
	public int[] getPointFeatures() {
		return pointFeatures;
	}

	public void setPointFeatures(int[] pointFeatures) {
		this.pointFeatures = pointFeatures;
	}
	public static SubtrajectoryGSON fromSubtrajectory(ISubtrajectory s, Description description){
		
		int number_of_point = s.getData().size();
		
		List<String> features_in_movelet = new ArrayList<>();
		
		int[] list_features = s.getPointFeatures();
		
		for(int i=0; i<=description.getPointComparisonDesc().getFeatureComparisonDesc().size(); i++) {
			
			if(ArrayUtils.contains(list_features, i))				
				features_in_movelet.add(description.getPointComparisonDesc().getFeatureComparisonDesc().get(i).getText());
			
		}
		
		List<HashMap<String, IFeature>> used_features = new ArrayList<>();
		
		for(int i=0; i<s.getData().size(); i++) {
			
			IPoint point = s.getData().get(i);
			
			HashMap<String, IFeature> features_in_point = new HashMap<>();
			
			for(String feature:features_in_movelet) {
				features_in_point.put(feature, point.getFeature(feature));
			}
			
			used_features.add(features_in_point);
		}
		
		
		return new SubtrajectoryGSON(s.getStart(), s.getEnd(), s.getTrajectory().getTid(), 
				s.getTrajectory().getLabel(), s.getFeatures(), s.getPointFeatures(), s.getSplitpoints(), s.getDistances(), s.getBestAlignments(),
				s.getQuality(), description, s.getData(), used_features);
		
	}
	
	public static SubtrajectoryGSON fromSubtrajectory2(ISubtrajectory s, Description description){
		
		int number_of_point = s.getData().size();
		
		List<String> features_in_movelet = new ArrayList<>();
		
		int[] list_features = s.getPointFeatures();
		
		for(int i=0; i<=description.getPointComparisonDesc().getFeatureComparisonDesc().size(); i++) {
			
			if(ArrayUtils.contains(list_features, i))				
				features_in_movelet.add(description.getPointComparisonDesc().getFeatureComparisonDesc().get(i).getText());
			
		}
		
		List<HashMap<String, IFeature>> used_features = new ArrayList<>();
		
		for(int i=0; i<s.getData().size(); i++) {
			
			IPoint point = s.getData().get(i);
			
			HashMap<String, IFeature> features_in_point = new HashMap<>();
			
			for(String feature:features_in_movelet) {
				features_in_point.put(feature, point.getFeature(feature));
			}
			
			used_features.add(features_in_point);
		}
				
		
		return new SubtrajectoryGSON(s.getStart(), s.getTrajectory().getTid(), s.getTrajectory().getLabel(),
				s.getPointFeatures(), s.getSplitpoints(), s.getDistances(), s.getBestAlignments(),
				s.getQuality(), used_features);
		
	}
	
	public ISubtrajectory toSubtrajectory(List<ITrajectory> trajectories){
		
		ITrajectory t = trajectories.stream().filter(e -> e.getTid() == this.trajectory).collect(Collectors.toList()).get(0);
		
		ISubtrajectory s = new Subtrajectory(start, end, t, pointFeatures, distances[0].length);
		
		LeftSidePureQuality lspq = new LeftSidePureQuality();
		lspq.setData(quality);		
		s.setDistances(distances);		
		s.setQuality(lspq);
		s.setSplitpoints(splitpoints);		
		
		return s;
		
	}
	

}
