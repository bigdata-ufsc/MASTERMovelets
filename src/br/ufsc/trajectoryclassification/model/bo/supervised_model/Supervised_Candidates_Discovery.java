package br.ufsc.trajectoryclassification.model.bo.supervised_model;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Callable;
import java.util.stream.IntStream;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;
import org.apache.commons.math3.stat.ranking.NaturalRanking;
import org.apache.commons.math3.stat.ranking.RankingAlgorithm;
import org.apache.commons.math3.util.Combinations;
import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsFilterAndRanker;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsFinding;
import br.ufsc.trajectoryclassification.model.bo.movelets.MyCounter;
import br.ufsc.trajectoryclassification.model.bo.movelets.SubtrajectoryBuilder;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;


public class Supervised_Candidates_Discovery implements Callable<Integer> {
	
	private List<ISubtrajectory> candidates;

	private ITrajectory trajectory;
	
	private List<ITrajectory> trajectories;
	
	private List<ITrajectory> trajectories_from_class;

	private IDistanceMeasureForSubtrajectory dmbt;
	
	private IQualityMeasure qualityMeasure;
	
	private int minSize;
	
	private int maxSize;
	
	private boolean cache;
	
	private boolean exploreDimensions;
	
	private int maxNumberOfFeatures = -1;
	
	private Random random;
	
	private RankingAlgorithm rankingAlgorithm = new NaturalRanking();
	
	public Supervised_Candidates_Discovery(List<ISubtrajectory> candidates, ITrajectory trajectory, List<ITrajectory> trajectories, List<ITrajectory> trajectories_from_class,
			IDistanceMeasureForSubtrajectory dmbt, IQualityMeasure qualityMeasure, int minSize, int maxSize, boolean cache, boolean exploreDimensions) {
		super();
		this.candidates = candidates;
		this.trajectory = trajectory;
		this.trajectories = trajectories;
		this.trajectories_from_class = trajectories_from_class;
		this.dmbt = dmbt;
		this.qualityMeasure = qualityMeasure;
		this.minSize = minSize;
		this.maxSize = maxSize;
		this.cache = cache;
		this.exploreDimensions = exploreDimensions;
		
		if (this.maxNumberOfFeatures < 1 || this.maxNumberOfFeatures > dmbt.getDMBP().getNumberOfFeatures())
			this.maxNumberOfFeatures = dmbt.getDMBP().getNumberOfFeatures();
				
	}
	
	public Supervised_Candidates_Discovery(List<ISubtrajectory> candidates, ITrajectory trajectory, List<ITrajectory> trajectories, List<ITrajectory> trajectories_from_class,
			IDistanceMeasureForSubtrajectory dmbt, IQualityMeasure qualityMeasure, int minSize, int maxSize, boolean cache, boolean exploreDimensions,
			int maxNumberOfFeatures) {
		this.candidates = candidates;
		this.trajectory = trajectory;
		this.trajectories = trajectories;
		this.trajectories_from_class = trajectories_from_class;
		this.dmbt = dmbt;
		this.qualityMeasure = qualityMeasure;
		this.minSize = minSize;
		this.maxSize = maxSize;
		this.cache = cache;
		this.exploreDimensions = exploreDimensions;
		this.maxNumberOfFeatures = maxNumberOfFeatures;

		switch (maxNumberOfFeatures) {
			case -1: this.maxNumberOfFeatures = dmbt.getDMBP().getNumberOfFeatures(); break;
			case -2: this.maxNumberOfFeatures = (int) Math.ceil(Math.log(dmbt.getDMBP().getNumberOfFeatures()))+1; break;
			default: break;
		}			
	}

	@Override
	public Integer call() throws Exception {		
	
		measureShapeletCollection();
	
		return 0;

	}

	public void measureShapeletCollection() {
		
		List<ISubtrajectory> candidates;
		
		/* Esta linha é muito importante para a reprodutibilidade de experimentos
		 * */
		this.random = new Random(trajectory.getTid());
		
		candidates = fastMoveletsDiscoveryUsingCache(trajectory, trajectories, minSize, maxSize, random);

		for (ISubtrajectory candidate : candidates) {
			
			/*
			 * STEP 1: COMPUTE DISTANCES, IF NOT COMPUTED YET
			 */
			if (candidate.getDistances() == null)		
				ComputeDistances(candidate);
			
//			 /* STEP 2: ASSES QUALITY
//			 */
//			if (qualityMeasure != null & candidate.getQuality() != null)
//				AssesQuality(candidate, random);
//			
//			if (candidate.getDistances_In_Class() == null)		
//				ComputeDistancesInClass(candidate);
//			
			/*
			 * STEP 2: COMPUTE THE PROPORTION
			 */
			
		}
//		candidates = MoveletsFilterAndRanker.rankCandidates(candidates);
//
//		candidates = MoveletsFilterAndRanker.getShapelets(candidates);
//		
		for (ISubtrajectory candidate : candidates)
			candidate.setProportionInClass(calculateProportion(candidate));
		
		
		this.candidates.addAll(candidates);
	
	}

	private double calculateProportion_AUX(ISubtrajectory candidate) {

		/*
		 * STEP 1: VERIFY WHICH ARE THE TRAJECTORIES THAT CONTAIN THAT CANDIDATE FOR EACH DIMENSION
		 */
		
		List<List<Integer>> trajectories_with_candidate = new ArrayList<>();
		
		for (int i = 0; i < candidate.getDistances_In_Class()[0].length; i++) {

			Integer j=0;

			List<Integer> one_dimension_coverage = new ArrayList<>();
			
			for(double[] distances:candidate.getDistances_In_Class()) {
				
				double distance = distances[i];
					
				if(distance==0.0)
					one_dimension_coverage.add(j);
					
				j++;
				
			}

			trajectories_with_candidate.add(one_dimension_coverage);
			
		}

		/*
		 * DEPRECATED STEP 2: TAKE THE INTERSECTION OF THE TRAJECTORIES THAT HAVE THAT CANDIDATE IN EACH DIMENSION
		 *
		
		List<Integer> all_with_candidate = trajectories_with_candidate.get(0);
		
		if(trajectories_with_candidate.size()>1) {
			
			for(int i=1; i<trajectories_with_candidate.size();i++) {
				
				List<Integer> some_with_candidate = trajectories_with_candidate.get(i);
				
				all_with_candidate.retainAll(some_with_candidate);
			}
		}*/
		
		/*
		 * STEP 2: TAKE THE PROPORTION IN EACH ATTRIBUTE, AND EXTRACT THE MEAN VALUE
		 */
		
		double proportions = 0.0;
		
		for(List<Integer> trajectories_per_dimension:trajectories_with_candidate) {
						
			int a = trajectories_per_dimension.size();
			int b = candidate.getDistances_In_Class().length;
			proportions += (double) a / b;
			
		}		
		
		/*
		 * STEP 3: CALCULATE THE PROPORTION
		 *
		
		int a = all_with_candidate.size();
		int b = candidate.getDistances_In_Class().length;
		double proportion =(double) a / b;
		*/
		/*
		 * STEP 3: IF THE CANDIDATE COVERS ONLY LESS THAN HALF OF THE TRAJECTORIES, THEN ABORT IT.
		 *
		if(proportion<0.5) {
			return -1.0;
		}*/
		return proportions/trajectories_with_candidate.size();
		
	}
	
	
	private double calculateProportion(ISubtrajectory candidate) {

		/*
		 * STEP 1: VERIFY WHICH ARE THE TRAJECTORIES THAT CONTAIN THAT CANDIDATE FOR EACH DIMENSION
		 */
		
		List<List<Integer>> trajectories_with_candidate = new ArrayList<>();
		
		for(double[] distances:candidate.getDistances()) {
			
			List<Integer> one_dimension_coverage = new ArrayList<>();
			Integer i=0;
			
			for(double distance:distances) {
				
				if(distance==0.0)
					one_dimension_coverage.add(i);
				
				i++;
			}
			
			trajectories_with_candidate.add(one_dimension_coverage);
		}

			
		/*
		 * STEP 2: CALCULATE THE PROPORTION
		 */
		
		double proportions = 0.0;
		
		for(List<Integer> trajectories_per_dimension:trajectories_with_candidate) {
						
			int a = trajectories_per_dimension.size();
			int b = this.trajectories_from_class.size();
			proportions += (double) a / b;
			
		}	
			
		double proportion = proportions/trajectories_with_candidate.size();
		
		/*
		 * STEP 3: IF THE CANDIDATE COVERS ONLY LESS THAN HALF OF THE TRAJECTORIES, THEN ABORT IT.
		 */
		
		if(proportion<0.5) {
			return -1.0;
		}
		
		return proportion;
		
	}

	private void ComputeDistances(ISubtrajectory candidate) {
		
		/* This pairs will store the subtrajectory of the best alignment 
		 * of the candidate into each trajectory and the distance 
		 * */
		Pair<ISubtrajectory,double[]> distance;
		
		double[][] trajectoryDistancesToCandidate = new double[candidate.getSplitpoints().length]
															  [trajectories.size()];
		
		ISubtrajectory[] bestAlignments = new ISubtrajectory[trajectories.size()];
				
		/* It calculates the distance of trajectories to the candidate
		 */
		for (int i = 0; i < trajectories.size(); i++) {
			
			distance = dmbt.getBestAlignment(candidate, trajectories.get(i));
						
			bestAlignments[i] = distance.getFirst();
			trajectoryDistancesToCandidate[i] = distance.getSecond();			
		}
		
		candidate.setDistances(trajectoryDistancesToCandidate);
		candidate.setBestAlignments(bestAlignments);
	}
	
	private void ComputeDistancesInClass(ISubtrajectory candidate) {
		
		/* This pairs will store the subtrajectory of the best alignment 
		 * of the candidate into each trajectory and the distance 
		 * */
		Pair<ISubtrajectory,double[]> distance;
		
		double[][] trajectoryDistancesToCandidate = new double[trajectories_from_class.size()][candidate.getSplitpoints().length];
		
		ISubtrajectory[] bestAlignments = new ISubtrajectory[trajectories_from_class.size()];
				
		/* It calculates the distance of trajectories to the candidate
		 */
		for (int i = 0; i < trajectories_from_class.size(); i++) {
			
			distance = dmbt.getBestAlignment(candidate, trajectories_from_class.get(i));
						
			bestAlignments[i] = distance.getFirst();
			trajectoryDistancesToCandidate[i] = distance.getSecond();			
		}
		
		candidate.setDistances_In_Class(trajectoryDistancesToCandidate);
	}

	private List<ISubtrajectory> getCandidatesUsingMDist(ITrajectory trajectory, List<ITrajectory> train, int size, double[][][][] mdist) {
		
		int n = trajectory.getData().size();
				
		List<ISubtrajectory> candidates = new ArrayList<>();

		for (int start = 0; start <= (n - size); start++) {
			
			int numberOfFeatures = dmbt.getDMBP().getNumberOfFeatures();
			
			List<ISubtrajectory> list = SubtrajectoryBuilder.build(start, start + size - 1, trajectory, numberOfFeatures, train.size(), exploreDimensions, maxNumberOfFeatures);
						
			double[][][] distancesForAllT = mdist[start];
			
			double distance;
			
			for (int i = 0; i < train.size(); i++) {
					
				double[][] distancesForT = distancesForAllT[i];
				
				double[][] ranksForT = new double[distancesForT.length][];
				
				int limit = train.get(i).getData().size() - size + 1;				
				
				if (limit > 0)
					for (int k = 0; k < numberOfFeatures; k++) {				
						ranksForT[k] = rankingAlgorithm.rank(Arrays.stream(distancesForT[k],0,limit).toArray());
					} // for (int k = 0; k < numberOfFeatures; k++)
				
				int k2 = 0;
				
				int currentFeatures;
						
				if (exploreDimensions){
					currentFeatures = 1;
				} else {
						currentFeatures = numberOfFeatures;
				}
				
				for (;currentFeatures <= maxNumberOfFeatures; currentFeatures++) {
					
					for (int[] comb : new Combinations(numberOfFeatures,currentFeatures)) {
												
						int bestPosition = (limit > 0) ? getBestAlignmentByRanking(ranksForT,comb) : -1;
												
						for (int j = 0; j < comb.length; j++) {
							
							distance = (bestPosition >= 0) ? distancesForT[comb[j]][bestPosition] : Double.MAX_VALUE;
							
							list.get(k2).getDistances()[j][i] = 
									(distance != Double.MAX_VALUE) ? Math.sqrt(distance / size) 
																   : Double.MAX_VALUE;
							
						} // for (int j = 0; j < comb.length; j++)
												
						k2++;
						
					} // for (int[] comb : new Combinations(numberOfFeatures,currentFeatures)) 					
					
				} // for (int i = 0; i < train.size(); i++)
				
			} // for (int currentFeatures = 1; currentFeatures <= numberOfFeatures; currentFeatures++)
			
			candidates.addAll(list);

		} // for (int start = 0; start <= (n - size); start++)
		
		return candidates;
		
	}
			
	private int getBestAlignmentByRanking(double[][] ranksForT, int[] comb) {
		
		double[] rankMerged = new double[ranksForT[0].length];
		for (int i = 0; i < comb.length; i++) {
			for (int j = 0; j < ranksForT[0].length; j++) {
				rankMerged[j] += ranksForT[comb[i]][j];
			}
		}
		/*			
		int minIdx = IntStream.range(0,rankMerged.length)
	            .reduce((i,j) -> rankMerged[i] > rankMerged[j] ? j : i)
	            .getAsInt(); 
	            */
		int minRankIndex = 0;
		for (int j = 1; j < rankMerged.length; j++) {
			if (rankMerged[j] < rankMerged[minRankIndex])
				minRankIndex = j;
		}
		/*
		System.out.println(Arrays.toString(rankMerged));
		System.out.println(minIdx);
		*/
		return minRankIndex;
	}

	public int getArrayIndex(double[] arr,double value) {
	    for(int i=0;i<arr.length;i++)
	        if(arr[i]==value) return i;
	    return -1;
	}
	
	private double[][][][] getBaseCase(ITrajectory trajectory, List<ITrajectory> train){
		int n = trajectory.getData().size();
		int size = 1;
		
		double[][][][] base = new double[(n - size)+1][][][];		
		
		for (int start = 0; start <= (n - size); start++) {
			
			base[start] = new double[train.size()][][];				
			
			for (int i = 0; i < train.size(); i++) {
								
				base[start][i] = new double[dmbt.getDMBP().getNumberOfFeatures()][(train.get(i).getData().size()-size)+1];
						
				for (int j = 0; j <= (train.get(i).getData().size()-size); j++) {
					
					double[] distance = dmbt.getDMBP().getDistance(
							trajectory.getData().get(start),
							train.get(i).getData().get(j));
							
					for (int k = 0; k < distance.length; k++) {
					
						base[start][i][k][j] = (distance[k] != Double.MAX_VALUE) ? (distance[k] * distance[k]) : Double.MAX_VALUE;					
					
					} // for (int k = 0; k < distance.length; k++)
					
				} // for (int j = 0; j <= (train.size()-size); j++)
				
			} //for (int i = 0; i < train.size(); i++)
			
		} // for (int start = 0; start <= (n - size); start++)

		return base;
	}


	private double[][][][] getNewSize(ITrajectory trajectory, List<ITrajectory> train, double[][][][] base, double[][][][] lastSize, int size) {
		
		int n = trajectory.getData().size();	
		
		for (int start = 0; start <= (n - size); start++) {
						
			for (int i = 0; i < train.size(); i++) {
				
				if (train.get(i).getData().size() >= size) {						
							
					for (int j = 0; j <= (train.get(i).getData().size()-size); j++) {
												
						for (int k = 0; k < lastSize[start][i].length; k++) {
							
							if (lastSize[start][i][k][j] != Double.MAX_VALUE)
								
								lastSize[start][i][k][j] += base[start+size-1][i][k][j+size-1];
						
						} // for (int k = 0; k < distance.length; k++) {
											
					} // for (int j = 0; j <= (train.size()-size); j++)
					
				} // if (train.get(i).getData().size() >= size) 
				
			} // for (int i = 0; i < train.size(); i++)
			
		} // for (int start = 0; start <= (n - size); start++)
		
		return lastSize;
	}

	
	private double[][][][] clone4DArray(double [][][][] source){
		double[][][][] dest = new double[source.length][][][];
		for (int i = 0; i < dest.length; i++) {
			dest[i] = new double[source[i].length][][];
			for (int j = 0; j < dest[i].length; j++) {
				dest[i][j] = new double[source[i][j].length][];
				for (int k = 0; k < dest[i][j].length; k++) {
					dest[i][j][k] = new double[source[i][j][k].length];
					for (int k2 = 0; k2 < source[i][j][k].length; k2++) {
						dest[i][j][k][k2] = source[i][j][k][k2];
					}
				}
			}
		}
		return dest;		
	}
	
	private List<ISubtrajectory> fastMoveletsDiscoveryUsingCache(ITrajectory trajectory, List<ITrajectory> train, int minSize, int maxSize, Random random){
				
		List<ISubtrajectory> candidates = new ArrayList<>();
		
		int n = trajectory.getData().size();
		
		//TO USE THE LOG, PUT "-Ms -3"
		
		switch (maxSize) {
			case -1: maxSize = n; break;
			case -2: maxSize = (int) Math.round( Math.log10(n) / Math.log10(2) ); break;	
			case -3: maxSize = (int) Math.ceil(Math.log(n))+1; break;	
			default: break;
		}
		
		MyCounter.numberOfCandidates += (maxSize * (maxSize-1) / 2);
		/* It starts with the base case
		 * */		
		int size = 1;

		Integer total_size = 0;
		
		double[][][][] base = getBaseCase(trajectory, train);
		
		if( minSize <= 1 ) {
			candidates.addAll(getCandidatesUsingMDist(trajectory, train, size, base));
//			candidates.forEach(x -> AssesQuality(x, random));
		}				
		
		double[][][][] lastSize = clone4DArray(base);			

		total_size = total_size + candidates.size();
		
		/* Tratar o resto dos tamanhos 
		 * */
		for (size = 2; size <= maxSize; size++) {
	
			/* Precompute de distance matrix
			 * */
			double[][][][] newSize = getNewSize(trajectory, train, base, lastSize, size);
			
			/* Create candidates and compute min distances
			 * */			
			List<ISubtrajectory> candidatesOfSize = getCandidatesUsingMDist(trajectory, train, size, newSize);
		
			total_size = total_size + candidatesOfSize.size();
//			
//			candidatesOfSize.forEach(x -> AssesQuality(x, random));
			candidates.addAll(candidatesOfSize);
		
			lastSize = newSize;
						
		} // for (int size = 2; size <= max; size++)	
	
		base =  null;
		lastSize = null;
		
		//System.out.println("Trajectory: " + trajectory.getTid() + ". Trajectory Size: " + trajectory.getData().size() + ". Number of Candidates: " + total_size + ". Total of Movelets: " + candidates.size() + ". Max Size: " + maxSize+ ". Used Features: " + maxNumberOfFeatures);
		
		return candidates;
	}


}
