package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Random;
import java.util.concurrent.Callable;

import org.apache.commons.math3.stat.ranking.NaturalRanking;
import org.apache.commons.math3.stat.ranking.RankingAlgorithm;
import org.apache.commons.math3.util.Combinations;
import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.bo.pivots.GetOutsidePivots;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Range;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;
import br.ufsc.trajectoryclassification.model.vo.Trajectory;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;

public class MoveletsDiscoveryPivots_TF_IDF implements Callable<Integer> {
	
	private List<ISubtrajectory> candidates;

	private ITrajectory trajectory;
	
	private List<ITrajectory> trajectories;

	private IDistanceMeasureForSubtrajectory dmbt;
	
	private IQualityMeasure qualityMeasure;
	
	private int minSize;
	
	private int maxSize;
	
	private boolean cache;
	
	private boolean exploreDimensions;
	
	private int maxNumberOfFeatures = 2;
	
	private Random random;
	
	private int limit_size = 0;
	
	private int porcentage = 10;
	
	private Boolean only_pivots = false;
	
	private List<String> csv_line;
	
	private List<String> class_line;
	
	private RankingAlgorithm rankingAlgorithm = new NaturalRanking();
			
	public MoveletsDiscoveryPivots_TF_IDF(List<ISubtrajectory> candidates, ITrajectory trajectory, List<ITrajectory> trajectories,
			IDistanceMeasureForSubtrajectory dmbt, IQualityMeasure qualityMeasure, int minSize, int maxSize, boolean cache, boolean exploreDimensions,
			int maxNumberOfFeatures, int porcentage, boolean only_pivots, List<String> csv_line, List<String> class_line) {
		this.candidates = candidates;
		this.trajectory = trajectory;
		this.trajectories = trajectories;
		this.dmbt = dmbt;
		this.qualityMeasure = qualityMeasure;
		this.minSize = minSize;
		this.maxSize = maxSize;
		this.cache = cache;
		this.exploreDimensions = exploreDimensions;
		this.maxNumberOfFeatures = maxNumberOfFeatures;
		this.porcentage = porcentage;
		this.only_pivots = only_pivots;
		this.csv_line = csv_line;
		this.class_line = class_line;

		if (this.maxNumberOfFeatures < 1 || this.maxNumberOfFeatures > dmbt.getDMBP().getNumberOfFeatures())
			this.maxNumberOfFeatures = dmbt.getDMBP().getNumberOfFeatures();

	}
	
	@Override
	public Integer call() throws Exception {
		
		measureShapeletCollection();
		
		return 0;
	}
	
	public void measureShapeletCollection() {

		
		List<ISubtrajectory> candidates = new ArrayList<>();
		
		this.random = new Random(trajectory.getTid());
		
		candidates = fastMoveletsDiscoveryUsingOutsidePivots(trajectory, trajectories, minSize, maxSize, random);

		for (ISubtrajectory candidate : candidates) {
			/*
			 * STEP 1: COMPUTE DISTANCES, IF NOT COMPUTED YET
			 */
			//if (candidate.getDistances() == null)		
			//	ComputeDistances(candidate);
			
			/*
			 * STEP 2: ASSES QUALITY, IF REQUIRED
			 */
			if (qualityMeasure != null & candidate.getQuality() != null)
				AssesQuality(candidate);
		}
		
		/* STEP 3: SELECTING BEST CANDIDATES
		 * */		
		this.candidates.addAll(MoveletsFilterAndRanker.getShapelets(candidates));
		//this.candidates.addAll(candidates);
	
	}
	
	private void AssesQuality(ISubtrajectory candidate) {
		qualityMeasure.assesQuality(candidate);
	}
	
	private void AssesQuality(ISubtrajectory candidate, Random random) {
		qualityMeasure.assesQuality(candidate, random);
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
	
	public Map<String,List<Integer>> getPOISInTrajectory(){

		Map<String,List<Integer>> pois_in_traj = new HashMap<String,List<Integer>>();
		
		for(int j=0; j<trajectory.getData().size(); j++) {
			
			String poi = trajectory.getData().get(j).getFeature("poi").toString();
			
			if(pois_in_traj.containsKey(poi)) {
				
				List<Integer> positions = pois_in_traj.get(poi);
				positions.add(j);
								
				pois_in_traj.put(poi, positions);
				
			}else {
				List<Integer> positions =  new ArrayList<>();
				positions.add(j);
				pois_in_traj.put(poi, positions);
			}
			
		}
		
		return pois_in_traj;
		
	}
	
	public Map<Double,String> getPOISInAscendingOrderFromCSV(){

		Map<Double,String> pois_in_csv = new HashMap<Double,String>();
		
		for(int j=1; j<csv_line.size(); j++) {
			if(Double.parseDouble(class_line.get(j))!=999.0)pois_in_csv.put(Double.parseDouble(class_line.get(j)), csv_line.get(j));
		}
		
		return pois_in_csv;
		
	}
	
	private Integer checkingGoodPointsInNeighborhood(Integer actual_position, List<Integer> pivots){
		
		if(actual_position + 1 >= trajectory.getData().size())			
			return actual_position;
		
		else if(pivots.contains(actual_position + 1))
			return checkingGoodPointsInNeighborhood(actual_position+1, pivots);
		
		else
			return actual_position+1;
		
	}
	
	private Map<Integer,List<Range>> getNeighbourhood(List<Integer> pivots){
		
		List<Integer> already_visited_points = new ArrayList<>();
		Map<Integer,List<Range>> map_of_ranges = new HashMap<Integer,List<Range>>();
		
		for(Integer point_position:pivots) {
			
			if(!already_visited_points.contains(point_position) || already_visited_points.isEmpty()) {
				
				Integer point_start = point_position - 1;
				Integer point_end = checkingGoodPointsInNeighborhood(point_position, pivots);

				if(point_position - 1 < 0 )
					point_start = 0;
				
				for(int j=point_start; j<=point_end; j++)
					already_visited_points.add(j);
				
				Range range = new Range(point_start, point_end, point_end-point_start+1);
				
				List<Range> key_range = map_of_ranges.get(point_end-point_start+1);
				
				if(key_range == null)
					key_range = new ArrayList<>();
				
				key_range.add(range);				
				map_of_ranges.put(point_end-point_start+1, key_range);
				
				//This one is necessary to get which is the max size of movelet it is possible to get
				//Using the pre computed ranges
				if(range.getMoveletSize() > limit_size)
					this.limit_size = range.getMoveletSize();
				
			}			
			
		}	
		
		return map_of_ranges;
	}	
	
	private Map<Integer,List<Range>> getNeighbourhoodOnlyPivots(List<Integer> pivots){
		
		List<Integer> already_visited_points = new ArrayList<>();
		Map<Integer,List<Range>> map_of_ranges = new HashMap<Integer,List<Range>>();
		
		for(Integer point_position:pivots) {
			
			if(!already_visited_points.contains(point_position) || already_visited_points.isEmpty()) {
				
				Integer point_start = point_position;
				Integer point_end = point_position;
				
				Range range = new Range(point_start, point_end, point_end-point_start+1);
				
				List<Range> key_range = map_of_ranges.get(point_end-point_start+1);
				
				if(key_range == null)
					key_range = new ArrayList<>();
				
				key_range.add(range);				
				map_of_ranges.put(point_end-point_start+1, key_range);
				
				//This one is necessary to get which is the max size of movelet it is possible to get
				//Using the pre computed ranges
				if(range.getMoveletSize() > limit_size)
					this.limit_size = range.getMoveletSize();
				
			}			
			
		}	
		
		return map_of_ranges;
	}	
	
	
	public List<Integer> FindPivots(){
		
		//First of all, we need to find the number of points that will become pivots
		int total_number_of_pivots = (int) Math.ceil(trajectory.getData().size() * ((double) porcentage/ (double) 100));
		int number_of_selected_pivots = 0;
		
		//We get all the POIs present on the Trajectory and in the CSV in ascending order
		Map<String,List<Integer>> pois_in_traj = getPOISInTrajectory();
		Map<Double,String> pois_in_csv = getPOISInAscendingOrderFromCSV();
				
		Double curr_order = 0.0;
		
		Collection<String> unique_pois_in_traj = pois_in_traj.keySet();

		//Get the position where the pivots are
		List<Integer> good_points = new ArrayList<>();
		
		//Now we iteratively select the points to be pivots, checking if the poi of higher score is in the trajectory.
		//If it is not, then we select the next, and so on.
		while(number_of_selected_pivots<=total_number_of_pivots) {
	
			String poi = pois_in_csv.get(curr_order);
			
			//get POI
			if(unique_pois_in_traj.contains(poi)) good_points.addAll(pois_in_traj.get(poi));
			
			curr_order++;
			
			number_of_selected_pivots = good_points.size();	
		}

		//Cut the extra points
		List<Integer> pivots = new ArrayList<>();
		for(int i=0; i<total_number_of_pivots;i++)pivots.add(good_points.get(i));
	
		return pivots;
		
	}
	
	public List<ITrajectory> getSubtrajectoriesFromRanges(Map<Integer,List<Range>> ranges_map){
		
		List<ITrajectory> neighborhood_subtrajectories = new ArrayList<>();
		
		for (Integer key : ranges_map.keySet()) {
			
			for(Range range:ranges_map.get(key)) {
				
				int start = range.getStart();
				List<IPoint> range_points = new ArrayList<>();
				
				for(int i =0; i<key;i++)range_points.add(trajectory.getData().get(start+i));									
				
				ITrajectory a_subtrajectory = new Trajectory(range_points, trajectory.getLabel());
				neighborhood_subtrajectories.add(a_subtrajectory);
				
			}
			
		}
		
		return neighborhood_subtrajectories;
		
	}
	
	public List<ISubtrajectory> fastMoveletsDiscoveryUsingOutsidePivots(ITrajectory trajectory, List<ITrajectory> train, int minSize, int maxSize, Random random){
		
		//GET THE PIVOTS		
		List<Integer> pivots = FindPivots();
		
		//GET THE NEIGHBORHOODS
		Map<Integer,List<Range>> ranges_map = null;
		
		if(only_pivots) ranges_map = getNeighbourhoodOnlyPivots(pivots);
		else ranges_map = getNeighbourhood(pivots);	

		//FOR EACH NEIGHBORHOOD, FIND THE CORRESPONDENT SUBTRAJECTORY
		List<ITrajectory> neighborhood_subtrajectories = getSubtrajectoriesFromRanges(ranges_map);
		
		List<double[][][][]> base_cases = new ArrayList<>();
		
		//CONSTRUCT THE BASE CASES
		for(int i =0; i<neighborhood_subtrajectories.size(); i++) base_cases.add(getBaseCase(neighborhood_subtrajectories.get(i), train));
		
		int total_size = 0;

		List<ISubtrajectory> final_candidates = new ArrayList<>();
		//FOR EACH BASE CASE, EXTRACT THE MOVELETS		
		
		for(int i = 0; i< base_cases.size(); i++) {

			int size = 1;
			ITrajectory trajectory_ = neighborhood_subtrajectories.get(i);
			
			List<ISubtrajectory> candidates = new ArrayList<>();
			
			if( minSize <= 1 ) {
				candidates.addAll(getCandidatesUsingMDist(trajectory_, train, size, base_cases.get(i)));
				candidates.forEach(x -> AssesQuality(x, random));
			}				
			
			double[][][][] lastSize = clone4DArray(base_cases.get(i));			

			total_size = total_size + candidates.size();
			
			int maxSize_ = trajectory_.getData().size();
			/* Tratar o resto dos tamanhos 
			 * */
			
			for (size = 2; size <= maxSize_; size++) {

				/* Precompute de distance matrix
				 * */
				double[][][][] newSize = getNewSize(trajectory_, train, base_cases.get(i), lastSize, size);
				
				/* Create candidates and compute min distances
				 * */			
				List<ISubtrajectory> candidatesOfSize = getCandidatesUsingMDist(trajectory_, train, size, newSize);
				
				total_size = total_size + candidatesOfSize.size();
				
				if (size >= minSize){
					
					//for (ISubtrajectory candidate : candidatesOfSize) AssesQuality(candidate);				
					candidatesOfSize.forEach(x -> AssesQuality(x, random));
					
					//candidatesOfSize = MoveletsFilterAndRanker.getShapelets(candidatesOfSize);
					
					candidates.addAll(candidatesOfSize);
				}
				
				lastSize = newSize;
							
			} // for (int size = 2; size <= max; size++)	
			
			lastSize = null;

			candidates = MoveletsFilterAndRanker.getShapelets(candidates);
			final_candidates.addAll(candidates);
		}


		System.out.println("Trajectory: " + trajectory.getTid() + ". Trajectory Size: " + trajectory.getData().size() + ". Number of Pivots: " + pivots.size() + ". Number of Ranges: " + neighborhood_subtrajectories.size() + ". Number of Candidates: " + total_size + ". Limit Size: " + limit_size + ". Total of Movelets: " + final_candidates.size());
		
		base_cases = null;
		return final_candidates;
		
	}

	
}
