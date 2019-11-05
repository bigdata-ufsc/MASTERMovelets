package br.ufsc.trajectoryclassification.model.bo.supervised_model;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadPoolExecutor;

import org.apache.commons.collections4.ListUtils;
import org.apache.commons.lang3.ArrayUtils;

import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsDiscovery;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsDiscoveryPivots;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsFilterAndRanker;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsFinding;
import br.ufsc.trajectoryclassification.model.bo.movelets.MyCounter;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.bo.pivots.GetOutsidePivots;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Range;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;
import br.ufsc.trajectoryclassification.utils.ProgressBar;
import br.ufsc.trajectoryclassification.utils.Utils;

/**
 * @author andres
 *
 */
public class MoveletsMultithread_Supervised {

	private List<ITrajectory> trainForMovelets;
	private List<ITrajectory> train;
	private List<ITrajectory> test;
	private IDistanceMeasureForSubtrajectory dmbt;
	private int minSize;
	private int maxSize = -1;
	private int nthreads;
	private IQualityMeasure qualityMeasure;
	private String resultDirPath;
	private boolean cache = false;
	private boolean last_prunning = false;
	private boolean showProgressBar = true;
	private boolean exploreDimensions = false;
	private static String medium = "none"; // Other values minmax, sd, interquartil
	private static String output = "numeric"; // Other values normalized and discretized
	private static int maxNumberOfFeatures = -1;	
	private String myclass = null;
	private static boolean pivots = false;
	private static Boolean attribute_limit = false;
	
	public MoveletsMultithread_Supervised(List<ITrajectory> trainForMovelets, List<ITrajectory> train, List<ITrajectory> test,
			IDistanceMeasureForSubtrajectory dmbt, int minSize, int nthreads,
			IQualityMeasure qualityMeasure, boolean cache, boolean exploreDimensions, String medium, String output,
			String resultDirPath, boolean last_prunning, boolean pivots, boolean attribute_limit) {
		super();
		this.trainForMovelets = trainForMovelets;
		this.train = train;
		this.test = test;
		this.dmbt = dmbt;
		this.minSize = minSize;
		this.nthreads = nthreads;
		this.qualityMeasure = qualityMeasure;
		this.cache = cache;
		this.exploreDimensions = exploreDimensions;
		this.medium = medium;
		this.last_prunning = last_prunning;
		this.output = output;
		this.pivots = pivots;
		this.attribute_limit = attribute_limit;
		this.resultDirPath = resultDirPath;

	}

	public static void setMaxNumberOfFeatures(int maxNumberOfFeatures) {
		MoveletsMultithread_Supervised.maxNumberOfFeatures = maxNumberOfFeatures;
	}
	
	public List<ISubtrajectory> supervised_Candidates_Discovery(List<ITrajectory> trajectories, List<ITrajectory> trajectories_from_class, boolean filterAndRank) {

		long startTime = System.nanoTime();
		List<ISubtrajectory> candidates = new ArrayList<>();
			
		ProgressBar progressBar = new ProgressBar("Supervised Candidates Discovery");
		int progress = 0;
		progressBar.update(progress, trajectories.size());


		
		for (ITrajectory trajectory : trajectories) {

			if ( myclass == null ||  trajectory.getLabel().equals(myclass)){
				
				Supervised_Candidates_Discovery shapeletsExtractor = new Supervised_Candidates_Discovery(candidates, trajectory, trajectories, trajectories_from_class, dmbt, (filterAndRank) ? qualityMeasure : null, minSize, maxSize, cache, exploreDimensions, maxNumberOfFeatures);
	
				shapeletsExtractor.measureShapeletCollection();
				
				progressBar.update(progress++, trajectories.size());
				
			}			
		}
		
		long endTime = System.nanoTime();

		long duration = (endTime - startTime)/1000000;
//		System.out.println("Time Spent: " + duration + "Antes de ordernar" + candidates.size());
		
		//return candidates;
		return candidates;
		
	}
	
	public List<ISubtrajectory> moveletsFinding(List<ISubtrajectory> candidates,
			List<ITrajectory> trajectories, boolean filterAndRank) {

		ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(this.nthreads);

		executor.setRejectedExecutionHandler(new ThreadPoolExecutor.DiscardPolicy());

		List<Future<Integer>> resultList = new ArrayList<>();

		/*
		 * Create groups of subtrajectory candidates to send to threads
		 */
		int groupSize = (candidates.size() > this.nthreads) ? Math.floorDiv(candidates.size(), this.nthreads) + 1 : 1;

		if (filterAndRank) Collections.shuffle(candidates);

		List<List<ISubtrajectory>> groups = ListUtils.partition(candidates, groupSize);

		/*
		 * Create the processes to send to thread
		 */
		for (List<ISubtrajectory> group : groups) {

			if (group.isEmpty())
				break;

			MoveletsFinding shapeletsExtractor = new MoveletsFinding(group, trajectories, dmbt);
			
			Future<Integer> result = executor.submit(shapeletsExtractor);

			resultList.add(result);
		}

		/*
		 * Execute threads
		 */
		List<Integer> results = new ArrayList<>();

		for (Future<Integer> future : resultList) {

			try {

				results.add(future.get());
				
				Executors.newCachedThreadPool();

			} catch (InterruptedException | ExecutionException e) {
				e.printStackTrace();
			}
		}

		executor.shutdown();

		/*
		 * Return the best candidates
		 */
		if (filterAndRank)
			return MoveletsFilterAndRanker.getShapelets(candidates);
		else
			return candidates;
	}

	public Map<Integer,List<Integer>> Identify_Relevant_Trajectorty_Parts(List<ITrajectory> trajectories_from_class, List<ISubtrajectory> candidates){
		
		Map<Integer,List<Integer>> relevant_parts = new HashMap<Integer,List<Integer>>();
		
		for(ITrajectory trajectory:trajectories_from_class)
			relevant_parts.put(trajectory.getTid(), new ArrayList<>());
		
		for(ISubtrajectory candidate:candidates) {
			
			int candidate_size = candidate.getSize();
			int candidate_start = candidate.getStart();

			for(int j=0; j<candidate_size;j++)
				if(!relevant_parts.get(candidate.getTrajectory().getTid()).contains(candidate_start+j)) {
					relevant_parts.get(candidate.getTrajectory().getTid()).add(candidate_start+j);
				}

		}
		
		return relevant_parts;
		
	}
	
	public List<HashMap<String, IFeature>> getDimensions(ISubtrajectory candidate) {
		
		List<String> features_in_movelet = new ArrayList<>();
		
		int[] list_features = candidate.getPointFeatures();
		
		for(int i=0; i<=dmbt.getDescription().getPointComparisonDesc().getFeatureComparisonDesc().size(); i++) {
			
			if(ArrayUtils.contains(list_features, i))				
				features_in_movelet.add(dmbt.getDescription().getPointComparisonDesc().getFeatureComparisonDesc().get(i).getText());
			
		}
		
		List<HashMap<String, IFeature>> used_features = new ArrayList<>();
		
		for(int i=0; i<candidate.getData().size(); i++) {
			
			IPoint point = candidate.getData().get(i);
			
			HashMap<String, IFeature> features_in_point = new HashMap<>();
			
			for(String feature:features_in_movelet) {
				features_in_point.put(feature, point.getFeature(feature));
			}
			
			used_features.add(features_in_point);
		}
		
		return used_features;
	}
	
	private boolean areEqual(List<HashMap<String, IFeature>> first, List<HashMap<String, IFeature>> second) {
		
		if (first.size() != second.size())
	        return false;
		
		if (first.get(0).size() != second.get(0).size())
	        return false;
	 
		for ( String key : first.get(0).keySet() ) {
			if(!second.get(0).containsKey(key)) {
		        return false;
		    }
		}
		
		boolean all_match = true;
		
		for(int i=0; i<first.size();i++) {
			
			HashMap<String, IFeature> f = first.get(i);
			HashMap<String, IFeature> s = second.get(i);
			
			if(f.entrySet().stream()
				      .allMatch(e -> e.getValue().equals(s.get(e.getKey()))))
				return false;
			
		}
	    return all_match;
	}
	
	public List<ISubtrajectory> moveletsDiscoveryMultithread(List<ITrajectory> trajectories, boolean filterAndRank, Map<Integer,List<Integer>> trajectory_parts) {

		long startTime = System.nanoTime();
		List<ISubtrajectory> candidates = new ArrayList<>();
					
		if (this.nthreads > 1){
			
		ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(this.nthreads);

		executor.setRejectedExecutionHandler(new ThreadPoolExecutor.DiscardPolicy());

		List<Future<Integer>> resultList = new ArrayList<>();
		
		for (ITrajectory trajectory : trajectories) {				
			
			if ( myclass == null ||  trajectory.getLabel().equals(myclass)){
					
				List<Integer> trajectory_part = trajectory_parts.get(trajectory.getTid());
				
				MoveletsDiscovery_Supervised shapeletsExtractor = new MoveletsDiscovery_Supervised(candidates, trajectory, trajectories, dmbt, (filterAndRank) ? qualityMeasure : null, minSize, maxSize, cache, exploreDimensions, maxNumberOfFeatures, trajectory_part, pivots, attribute_limit);
			
				Future<Integer> result = executor.submit(shapeletsExtractor);

				resultList.add(result);
					
			}
			
		}
		
		ProgressBar progressBar = new ProgressBar("Movelet Discovery");

		int progress = 0;
		progressBar.update(progress, trajectories.size());
		
		List<Integer> results = new ArrayList<>();

		for (Future<Integer> future : resultList) {

			try {

				results.add(future.get());
				
				progressBar.update(progress++, trajectories.size());

				Executors.newCachedThreadPool();

			} catch (InterruptedException | ExecutionException e) {
				e.printStackTrace();
			}
		}

		executor.shutdown();
		}
		
		else {
			
			ProgressBar progressBar = new ProgressBar("Movelet Discovery");
			int progress = 0;
			progressBar.update(progress, trajectories.size());
						
			for (ITrajectory trajectory : trajectories) {

				List<Integer> trajectory_part = trajectory_parts.get(trajectory.getTid());
				MoveletsDiscovery_Supervised shapeletsExtractor = new MoveletsDiscovery_Supervised(candidates, trajectory, trajectories, dmbt, (filterAndRank) ? qualityMeasure : null, minSize, maxSize, cache, exploreDimensions, maxNumberOfFeatures, trajectory_part, pivots, attribute_limit);
				
				shapeletsExtractor.measureShapeletCollection();
				
				progressBar.update(progress++, trajectories.size());
				
			}			
			
		}

		long endTime = System.nanoTime();

		long duration = (endTime - startTime)/1000000;
//		System.out.println("Time Spent: " + duration + "Antes de ordernar" + candidates.size());
		
		if (filterAndRank)
			return MoveletsFilterAndRanker.rankCandidates(candidates);
		else
			return candidates;
		
	}
	
	public void run() {

		long startTime = System.currentTimeMillis();
		
		System.out.println("Movelets generation with Log limit.");
		/* STEP 1: We need to extract the candidates from a class
		 * */
		
		List<ITrajectory> trajectories_from_class = new ArrayList<>(); 
		
		for (ITrajectory trajectory : trainForMovelets)
			if ( myclass == null ||  trajectory.getLabel().equals(myclass))				
				trajectories_from_class.add(trajectory);

		boolean removeSelfsimilarAndRank = true;
		List<ISubtrajectory> candidates = supervised_Candidates_Discovery(trajectories_from_class, trajectories_from_class, removeSelfsimilarAndRank);
		
		/* STEP 2: SELECT ONLY CANDIDATES WITH PROPORTION >50%
		 * */	

		List<ISubtrajectory> orderedCandidates = new ArrayList<>();

		for(ISubtrajectory candidate:candidates)
			if(candidate.getProportionInClass()>0)
				orderedCandidates.add(candidate);
		

		int add = 0;

		List<ISubtrajectory> best_candidates = new ArrayList<>();
		
		/* STEP 3: SORT THE CANDIDATES BY PROPORTION VALUE
		 * */		
		
		orderedCandidates.sort(new Comparator<ISubtrajectory>() {
			@Override
			public int compare(ISubtrajectory o1, ISubtrajectory o2) {
				
				return (-1) * Double.compare(o1.getProportionInClass(),o2.getProportionInClass());			
				
			}
		});
		
		/* STEP 4: IDENTIFY EQUAL CANDIDATES
		 * */	

		int[] attribute_usage = new int [dmbt.getDMBP().getNumberOfFeatures()]; // array of 5 ints
		
		for(ISubtrajectory candidate:orderedCandidates) {
			
			if(best_candidates.isEmpty())
				best_candidates.add(candidate);
			else {
				boolean equal = false;
				for(ISubtrajectory best_candidate:best_candidates) {
					
					List<HashMap<String, IFeature>> used_features_c1 = getDimensions(candidate);
					List<HashMap<String, IFeature>> used_features_c2 = getDimensions(best_candidate);
					
					if(used_features_c1.size()==used_features_c2.size())
						if(areEqual(used_features_c1, used_features_c2)) {
							equal = true;
							break;
						}
					
				}
				if(!equal) {
					best_candidates.add(candidate);
					attribute_usage[candidate.getPointFeatures().length-1]++;
				}
			}
				
			
		}
		
		if(attribute_limit) {
			
			int max=0;
			int number_of_features =-1;
			
			for(int j=0; j<attribute_usage.length; j++) {
				
				if(attribute_usage[j]>max) {
					max = attribute_usage[j];
					number_of_features = j+1;
				}
				
			}
			
			setMaxNumberOfFeatures(number_of_features);
			
		}

		/* STEP 5: IDENTIFY THE TRAJECTORY POINTS THAT HAVE INTERESTING CONTENT
		 * */	
		Map<Integer,List<Integer>> trajectory_parts = Identify_Relevant_Trajectorty_Parts(trajectories_from_class, best_candidates);
		
		
		/* SELECT ONLY HALF OF THE CANDIDATES
		List<ISubtrajectory> half_ordered_candidates = orderedCandidates.subList(0, (int) Math.ceil((double) orderedCandidates.size()/(double) 2));
		* */
		 
		List<ISubtrajectory> movelets = moveletsDiscoveryMultithread(trainForMovelets, removeSelfsimilarAndRank,trajectory_parts);
		System.out.println(Utils.summaryMovelets(movelets));
		
		long estimatedTime = System.currentTimeMillis() - startTime;
		
		MyCounter.data.put("MoveletsDiscoveryTime", estimatedTime);
		MyCounter.data.put("MoveletsFound", (long) movelets.size());
		
		MyCounter.numberOfShapelets = movelets.size();

		/* STEP 2: It runs the pruning process  
		 * */		

		if(last_prunning)
			movelets = MoveletsFilterAndRanker.noveltyFilter(movelets);
		
		MyCounter.data.put("MoveletsAfterPruning", (long) movelets.size());
		
		System.out.println(Utils.summaryMovelets(movelets));
		
		/* STEP 3: It transforms the training and test sets of trajectories
		 * using the movelets
		 * */
				
		/* It initializes the set of distances of all movelets to null
		 * */
		//System.out.println(Arrays.toString( movelets.get(0).getDistances()[0] ));
		for (ISubtrajectory movelet : movelets) {
			movelet.setDistances(null);
		}
				
		/* Transforming the training set using movelets
		 * In this step the set of distances is filled by this method
		 * */
		new MoveletsFinding(movelets,train,dmbt).run();
		//System.out.println(Arrays.toString( movelets.get(0).getDistances()[0] ));
		
		/* It puts distances as trajectory attributes
		 * */
		
		for (ISubtrajectory movelet : movelets) {
			Utils.putAttributeIntoTrajectories(train, movelet, output, medium);
		}		
		
		
		/* It writes a JSON and a CSV in a attribute-value format
		 * */		
		Utils.writeAttributesCSV(train, resultDirPath + "train.csv");
		
		Utils.writeShapeletsToGSON(train, movelets, dmbt.getDescription(), resultDirPath + "moveletsOnTrain.json");		
		
		/* If a test trajectory set was provided, it does the same.
		 * and return otherwise 
		 */		
		if (test.isEmpty()) return;
		
		for (ISubtrajectory movelet : movelets) {
			movelet.setDistances(null);
		}
		
		new MoveletsFinding(movelets,test,dmbt).run();				

		for (ISubtrajectory movelet : movelets) {
			Utils.putAttributeIntoTrajectories(test, movelet, output, medium);
		}
		
		Utils.writeAttributesCSV(test, resultDirPath + "test.csv");
	}
	
		
	public void setShowProgressBar(boolean showProgressBar) {
		this.showProgressBar = showProgressBar;
	}

	public void setMaxSize(int maxSize) {
		this.maxSize = maxSize;
	}

	public void setClass(String myclass) {
		// TODO Auto-generated method stub
		this.myclass = myclass;
	}

}
