package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
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

import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.bo.pivots.GetOutsidePivots;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;
import br.ufsc.trajectoryclassification.utils.ProgressBar;
import br.ufsc.trajectoryclassification.utils.Utils;

/**
 * @author andres
 *
 */
public class MoveletsMultithread {

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
	private boolean pivots = false;
	private boolean last_prunning = false;
	private boolean showProgressBar = true;
	private boolean exploreDimensions = false;
	private static String medium = "none"; // Other values minmax, sd, interquartil
	private static String output = "numeric"; // Other values normalized and discretized
	private static int moveletsPerTrajectory = -1; // Filtering
	private static int maxNumberOfFeatures = -1;	
	private int porcentage=10;
	private Boolean only_pivots = false;
	private String myclass = null;
	private List<String> csv_line = null;
	private List<String> class_line = null;
	
	public MoveletsMultithread(List<ITrajectory> trainForMovelets, List<ITrajectory> train, List<ITrajectory> test,
			IDistanceMeasureForSubtrajectory dmbt, int minSize, int nthreads,
			IQualityMeasure qualityMeasure, boolean cache, boolean exploreDimensions, String medium, String output, int moveletsPerTrajectory,
			String resultDirPath, boolean pivots, boolean last_prunning, int porcentage, boolean only_pivots) {
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
		this.pivots = pivots;
		this.last_prunning = last_prunning;
		this.output = output;
		this.moveletsPerTrajectory = moveletsPerTrajectory;		
		this.resultDirPath = resultDirPath;
		this.porcentage = porcentage;
		this.only_pivots = only_pivots;

	}
	
	public MoveletsMultithread(List<ITrajectory> trainForMovelets, List<ITrajectory> train, List<ITrajectory> test,
			IDistanceMeasureForSubtrajectory dmbt, int minSize, int nthreads,
			IQualityMeasure qualityMeasure, boolean cache, boolean exploreDimensions, String medium, String output, int moveletsPerTrajectory,
			String resultDirPath, boolean pivots, boolean last_prunning, int porcentage, boolean only_pivots, List<String> csv_line, List<String> class_line) {
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
		this.pivots = pivots;
		this.last_prunning = last_prunning;
		this.output = output;
		this.moveletsPerTrajectory = moveletsPerTrajectory;		
		this.resultDirPath = resultDirPath;
		this.porcentage = porcentage;
		this.only_pivots = only_pivots;
		this.csv_line = csv_line;
		this.class_line = class_line;

	}

	public static void setMaxNumberOfFeatures(int maxNumberOfFeatures) {
		MoveletsMultithread.maxNumberOfFeatures = maxNumberOfFeatures;
	}
	
	public List<ISubtrajectory> moveletsDiscoveryMultithread(List<ITrajectory> trajectories, boolean filterAndRank) {

		long startTime = System.nanoTime();
		List<ISubtrajectory> candidates = new ArrayList<>();
					
		if (this.nthreads > 1){
			
		ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(this.nthreads);

		executor.setRejectedExecutionHandler(new ThreadPoolExecutor.DiscardPolicy());

		List<Future<Integer>> resultList = new ArrayList<>();
		

		for (ITrajectory trajectory : trajectories) {				
			
			if ( myclass == null ||  trajectory.getLabel().equals(myclass)){

				if(pivots) {
					
					MoveletsDiscoveryPivots shapeletsExtractor = new MoveletsDiscoveryPivots(candidates, trajectory, trajectories, dmbt, (filterAndRank) ? qualityMeasure : null, minSize, maxSize, cache, exploreDimensions, maxNumberOfFeatures, porcentage, only_pivots);
					//MoveletsDiscoveryPivots_TF_IDF shapeletsExtractor = new MoveletsDiscoveryPivots_TF_IDF(candidates, trajectory, trajectories, dmbt, (filterAndRank) ? qualityMeasure : null, minSize, maxSize, cache, exploreDimensions, maxNumberOfFeatures, porcentage, only_pivots, csv_line, class_line);
					
					Future<Integer> result = executor.submit(shapeletsExtractor);

					resultList.add(result);
				
				}
				else{
					
					MoveletsDiscovery shapeletsExtractor = new MoveletsDiscovery(candidates, trajectory, trajectories, dmbt, (filterAndRank) ? qualityMeasure : null, minSize, maxSize, cache, exploreDimensions, maxNumberOfFeatures);
				
					Future<Integer> result = executor.submit(shapeletsExtractor);

					resultList.add(result);
				
				}
					
			}
			
		}
		
		//ProgressBar progressBar = new ProgressBar("Movelet Discovery");

		int progress = 0;
		//progressBar.update(progress, trajectories.size());
		
		List<Integer> results = new ArrayList<>();

		for (Future<Integer> future : resultList) {

			try {

				results.add(future.get());
				
				//progressBar.update(progress++, trajectories.size());

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
				
				
				if(pivots) {
					
					MoveletsDiscoveryPivots shapeletsExtractor = new MoveletsDiscoveryPivots(candidates, trajectory, trajectories, dmbt, (filterAndRank) ? qualityMeasure : null, minSize, maxSize, cache, exploreDimensions, maxNumberOfFeatures, porcentage, only_pivots);

					shapeletsExtractor.measureShapeletCollection();

					//progressBar.update(progress++, trajectories.size());
				
				}
				else{
					
					MoveletsDiscovery shapeletsExtractor = new MoveletsDiscovery(candidates, trajectory, trajectories, dmbt, (filterAndRank) ? qualityMeasure : null, minSize, maxSize, cache, exploreDimensions, maxNumberOfFeatures);

					shapeletsExtractor.measureShapeletCollection();
					
					//progressBar.update(progress++, trajectories.size());
				
				}
				
			}			
			
		}

		long endTime = System.nanoTime();

		long duration = (endTime - startTime)/1000000;
		//System.out.println("Time Spent: " + duration + "Antes de ordernar" + candidates.size());
		
		if (filterAndRank)
			return MoveletsFilterAndRanker.rankCandidates(candidates);
		else
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

	public void run() {

		long startTime = System.currentTimeMillis();
		
		Date starting_date = new Date();
		//System.out.println(starting_date);
		
		/* STEP 1: It starts at discovering movelets 
		 * */		
		boolean removeSelfsimilarAndRank = true;
		
		List<ISubtrajectory> movelets = moveletsDiscoveryMultithread(trainForMovelets, removeSelfsimilarAndRank);

		Date ending_date = new Date();
		//System.out.println(ending_date);
		
		//System.out.println(Utils.summaryMovelets(movelets));
		
		long estimatedTime = System.currentTimeMillis() - startTime;
		
		MyCounter.data.put("MoveletsDiscoveryTime", estimatedTime);
		MyCounter.data.put("MoveletsFound", (long) movelets.size());
		
		MyCounter.numberOfShapelets = movelets.size();

		/* STEP 2: It runs the pruning process  
		 * */		

		if(last_prunning)
			movelets = MoveletsFilterAndRanker.noveltyFilter(movelets);
		
		MyCounter.data.put("MoveletsAfterPruning", (long) movelets.size());
		
		//System.out.println(Utils.summaryMovelets(movelets));
		
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

		//Utils.writeShapeletsToGSON(test, movelets, dmbt.getDescription(), resultDirPath + "moveletsOnTest.json");
		
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
