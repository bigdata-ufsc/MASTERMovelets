package br.ufsc.trajectoryclassification.view.movelets.run;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.apache.commons.io.FilenameUtils;

import br.ufsc.trajectoryclassification.model.bo.featureExtraction.ComparisonFeatureParameterEstimation;
import br.ufsc.trajectoryclassification.model.bo.dmbs.DMS;
import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.PointFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.featureExtraction.parsers.TrajectoryFeaturesExtraction;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsMultithread;
import br.ufsc.trajectoryclassification.model.bo.movelets.MyCounter;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.InformationGain;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureCV;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureCVLigth;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.LeftSidePureOld;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.Unsupervised1CV;
import br.ufsc.trajectoryclassification.model.bo.pivots.GetOutsidePivots;
import br.ufsc.trajectoryclassification.model.bo.supervised_model.MoveletsDiscovery_Supervised;
import br.ufsc.trajectoryclassification.model.dao.TrajectoryDAO;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.description.Description;
import br.ufsc.trajectoryclassification.model.vo.description.FeatureComparisonDesc;
import br.ufsc.trajectoryclassification.model.vo.features.FoursquareVenue;
import br.ufsc.trajectoryclassification.model.vo.features.GowallaCheckin;
import br.ufsc.trajectoryclassification.model.vo.features.Nominal;
import br.ufsc.trajectoryclassification.model.vo.features.ProvidedDistances;
import br.ufsc.trajectoryclassification.utils.Utils;

public class MoveletsRunUnit {

	private static String CURRENT_DIR = null;
	private static String RESULT_DIR = null;
	private static String DESCRIPTION_FILE = null;	
	private static String PIVOTS_FILE = null;	
	private static int nthreads = 1;
	private static int minSize = 2;
	private static boolean pivots = false;
	private static int maxSize = -1; // unlimited maxSize
	private static String strQualityMeasure = "LSP"; 	
	private static boolean cache = true;
	private static boolean exploreDimensions = false;
	private static int maxNumberOfFeatures = -1;
	private static int samples = 1;
	private static boolean outside_pivots = false;
	private static boolean last_prunning = false;
	private static double sampleSize = 1;
	private static String medium = "none"; // Other values minmax, sd, interquartil
	private static String output = "numeric"; // Other values numeric discretized
	private static int moveletsPerTrajectory = -1; // Filtering
	private static boolean lowMemory = false;
	private static int pivot_porcentage=10;
	private static Boolean only_pivots = false;
	
	public static void configure(String[] args) {

		for (int i = 0; i < args.length; i = i + 2) {
			String key = args[i];
			String value = args[i + 1];
			switch (key) {
			case "-curpath":
				CURRENT_DIR = value;
				break;
			case "-respath":
				RESULT_DIR = value;
				break;
			case "-descfile":
				DESCRIPTION_FILE = value;
				break;
			case "-outside_pivots":
				PIVOTS_FILE = value;
				break;
			case "-nt":
				nthreads = Integer.valueOf(value);
				break;
			case "-ms":
				minSize = Integer.valueOf(value);
				break;
			case "-Ms":
				maxSize = Integer.valueOf(value);
				break;
			case "-q":
				strQualityMeasure = value;
				break;
			case "-cache":
				cache = Boolean.valueOf(value);				
				break;
			case "-ed":
				exploreDimensions = Boolean.valueOf(value);				
				break;					
			case "-mnf":
				maxNumberOfFeatures = Integer.valueOf(value);		
				break;
			case "-samples":
				samples = Integer.valueOf(value);				
				break;					
			case "-sampleSize":
				sampleSize = Double.valueOf(value);				
				break;					
			case "-medium":
				medium = value;				
				break;			
			case "-output":
				output = value;				
				break;		
			case "-pvt":
				pivots = Boolean.valueOf(value);				
				break;	
			case "-mpt":
				moveletsPerTrajectory = Integer.valueOf(value);				
				break;
			case "-lowm":
				lowMemory = Boolean.valueOf(value);				
				break;			
			case "-lp":
				last_prunning = Boolean.valueOf(value);				
				break;	
			case "-pp":
				pivot_porcentage = Integer.valueOf(value);
				break;
			case "-op":
				only_pivots = Boolean.valueOf(value);
				break;
			default:
				System.err.println("Par�metro " + key + " inv�lido.");
				System.exit(1);
				return;
			}
		}

	}
	
	
	public static String showConfiguration() {

		String str = new String();
		
		if(pivots)
			str += "Starting running Movelet_pivots extractor (10%) " + System.getProperty("line.separator");
		else if(maxSize==-3)
			str += "Starting running MASTERMovelets_Log extractor " + System.getProperty("line.separator");
		else
			str += "Starting running MASTERMovelets extractor " + System.getProperty("line.separator");
		
		if (PIVOTS_FILE != null)
			str += "Getting pivots from outside file:" + PIVOTS_FILE + System.getProperty("line.separator");
		
		str += "Configurations:" + System.getProperty("line.separator");

		str += "\tDatasets directory:	    " + CURRENT_DIR + System.getProperty("line.separator");

		str += "\tResults directory:    " + RESULT_DIR + System.getProperty("line.separator");
		
		str += "\tDescription file :    " + DESCRIPTION_FILE + System.getProperty("line.separator");

		str += "\tAllowed Threads:      " + nthreads + System.getProperty("line.separator");

		str += "\tMin size:             " + minSize + System.getProperty("line.separator");

		str += "\tMax size:             " + maxSize + System.getProperty("line.separator");

		str += "\tQuality Measure:      " + strQualityMeasure + System.getProperty("line.separator");
		
		str += "\tExplore dimensions:   " + exploreDimensions + System.getProperty("line.separator");

		str += "\tSamples:      " + samples + System.getProperty("line.separator");
		
		str += "\tSample Size:   " + sampleSize + System.getProperty("line.separator");
		
		str += "\tMedium:   " + medium + System.getProperty("line.separator");
		
		str += "\tMPT:   " + moveletsPerTrajectory + System.getProperty("line.separator");
		
		str += "\tOutput:   " + output + System.getProperty("line.separator");
		
		if(last_prunning)
			str += "WITH LAST PRUNNING" + System.getProperty("line.separator");
		else
			str += "WITHOUT LAST PRUNNING" + System.getProperty("line.separator");
		
		return str;

	}

	public static void main(String[] args) {
		
		 Date starting_date = new Date();
		 //System.out.println(starting_date);
		
		if (args.length == 0) return;
		/*
		 * STEP 1. Configura par�metros de entrada
		 */
		configure(args);
		//System.out.println(showConfiguration());	
		
		if (DESCRIPTION_FILE == null) return;
		
		String DESCRIPTION_FILE_NAME = FilenameUtils.removeExtension(
				new File(DESCRIPTION_FILE).getName());
		
		if (exploreDimensions)
			DESCRIPTION_FILE_NAME += "_ED"; 
		
		if (maxNumberOfFeatures!=-1)
			DESCRIPTION_FILE_NAME += "_MaxNumberOfFeatures_" +maxNumberOfFeatures; 
		
		String resultDirPath = "";
		
		

		if (PIVOTS_FILE != null) {
			resultDirPath = RESULT_DIR + "/MASTERMoveletsPivots_BOW/" + DESCRIPTION_FILE_NAME;
			outside_pivots = true;
		}
		else {
			if(pivots)
				resultDirPath = RESULT_DIR + "/MASTERMoveletsPivots/" + DESCRIPTION_FILE_NAME + "/Porcentage_" + pivot_porcentage;
			else {
				if(maxSize==-3) {
					resultDirPath = RESULT_DIR + "/MASTERMovelets_LOG/" + DESCRIPTION_FILE_NAME;
				}else
					resultDirPath = RESULT_DIR + "/MASTERMovelets/" + DESCRIPTION_FILE_NAME;
			}
		}
	
		
		if(last_prunning)
			resultDirPath = resultDirPath + "_With_Last_Prunning/";
		else
			resultDirPath = resultDirPath + "_Witout_Last_Prunning/";

		
		String trainDirPath = CURRENT_DIR + "/train";
		String testDirPath = CURRENT_DIR + "/test";
						
		String descriptionPathFile = DESCRIPTION_FILE;
				
		//System.out.println("\nStarting...");
				
		/* Load description file and train and test trajectories */
		Description description = new TrajectoryDAO().loadDescription(descriptionPathFile);
		
		/* If you choose for providing distances in a file here we are going to load them */
		loadProvidedDistances(description);
		
		List<ITrajectory> train = Utils.loadTrajectories(trainDirPath, description);
						
		if (train.isEmpty()) {
			System.out.println("Empty training set");
			return;
		}
		
		
		PointFeaturesExtraction.fillAllTrajectories(train, description);
		TrajectoryFeaturesExtraction.fillAllTrajectories(train,description);
		//Utils.writeTrajectoriesToGSON(train, description, resultDirPath + "trainAfterFeatureExtraction.json");
			
		List<ITrajectory> trainForMovelets = train;
						
		if (description.getSubtrajectoryComparisonDesc() == null){
			Utils.writeAttributesCSV(train, resultDirPath + "train.csv");
		}
	
		List<ITrajectory> test = Utils.loadTrajectories(testDirPath, description);
				
		if (!test.isEmpty()){			
			PointFeaturesExtraction.fillAllTrajectories(test, description);		
			TrajectoryFeaturesExtraction.fillAllTrajectories(test,description);
			//Utils.writeTrajectoriesToGSON(test, description, resultDirPath + "testAfterFeatureExtraction.json");
			
			if (description.getSubtrajectoryComparisonDesc() == null){
				Utils.writeAttributesCSV(test, resultDirPath + "test.csv");
				return;
			}
		}
		
		List<String> classes = train.stream().map(e -> (String) e.getLabel()).distinct().collect(Collectors.toList());	
		
		IDistanceMeasureForSubtrajectory dms =  DMS.getDMSfromDescription(description);
		
		IQualityMeasure qualityMeasure = new LeftSidePureCVLigth(trainForMovelets,samples,sampleSize,medium);
		
		long startTime = System.currentTimeMillis();
		for (String myclass : classes) {			
			
			if ( ! (new File(resultDirPath + myclass + "/test.csv").exists()) ) {
			
				//System.out.println("Class: " + myclass + ". Discovering movelets.");
				
				MoveletsMultithread analysis = new MoveletsMultithread(
						trainForMovelets, train, test, dms, minSize, nthreads, qualityMeasure, cache, exploreDimensions, medium, output, moveletsPerTrajectory, 
						resultDirPath + myclass + "/" , pivots, last_prunning, pivot_porcentage, only_pivots);
				
				if(outside_pivots) {
					
					System.out.println("Getting Pivots from Outside.");
					
					GetOutsidePivots outside_pivots = new GetOutsidePivots();
					
					List<List<String>> csv_lines = outside_pivots.readCSV(PIVOTS_FILE);

					List<String> tfidf_header = outside_pivots.getTFIDFHeader(csv_lines);
					
					List<String> class_line = outside_pivots.getClassLine(Double.parseDouble(myclass), csv_lines);
					
					analysis = new MoveletsMultithread(
							trainForMovelets, train, test, dms, minSize, nthreads, qualityMeasure, cache, exploreDimensions, medium, output, moveletsPerTrajectory, 
							resultDirPath + myclass + "/" , pivots, last_prunning, pivot_porcentage, only_pivots, tfidf_header, class_line);
				}				
				
				train.forEach(e ->  e.getFeatures().clear());
				train.forEach(e ->  e.getAttributes().clear());
				test.forEach(e ->  e.getFeatures().clear());
				test.forEach(e ->  e.getAttributes().clear());
		     	
				analysis.setMaxNumberOfFeatures(maxNumberOfFeatures);
				analysis.setClass(myclass);
				analysis.setMaxSize(maxSize);				
				analysis.run();
				
				System.gc();				
			} else {
				
				System.out.println("Class: " + myclass + ". Movelets previously discovered.");
			}
			
		}
		
		MyCounter.data.put("candidates", MyCounter.numberOfCandidates);
				
		System.out.println(MyCounter.data);
		
		long endTime = System.currentTimeMillis();
		
		System.out.println("Processing time: " + (endTime - startTime) + " milliseconds" );
		Date end_date = new Date();
		System.out.println(end_date);
			
	}


	private static List<ITrajectory> getStratifiedTrainData(List<ITrajectory> train, double p) {
		// TODO Auto-generated method stub
		List<String> classes = train.stream().map(e -> (String) e.getLabel()).distinct().collect(Collectors.toList());
		
		Map<String, List<ITrajectory> > grouped = train.stream()
	            .collect(Collectors.groupingBy(ITrajectory::getLabel, Collectors.toList()));
			  
		Random seed = new Random(1);
		
		List<ITrajectory> newtrain = new ArrayList<>();
		
		for (String myclass : classes) {			
			
			List<ITrajectory> list = grouped.get(myclass);
			int n1 = (int) Math.round(list.size() * p);
			if (n1 == 0) n1 = 1;
			Collections.shuffle(list,seed);
			list = list.subList(0, n1);			
			newtrain.addAll(list);		
		}
		
		return  newtrain;
	}


	private static void loadProvidedDistances(Description description) {
		// TODO Auto-generated method stub
		
		List<FeatureComparisonDesc> featureComparisonDescs = description.getPointComparisonDesc().getFeatureComparisonDesc();
		
		Map<String, ProvidedDistances> providedDistances = Nominal.getProvidedDistances();
		
		for ( FeatureComparisonDesc fcd : featureComparisonDescs) {
			
			if (fcd.getDistance().equalsIgnoreCase("provided")){
				
				String name = fcd.getText();
				String vocabularyPath = CURRENT_DIR + "/" + (String) fcd.getAdditionalProperties().get("vocabularyFile");
				String distancesPath  = CURRENT_DIR + "/" + (String) fcd.getAdditionalProperties().get("distancesFile");
				
				System.out.println("Loading provided distances for feature: \""+ name +"\"" +
								   "\n\tfrom "+ vocabularyPath +
								   "\n\t and "+ distancesPath);
				
				ProvidedDistances pd = new ProvidedDistances(vocabularyPath, distancesPath);
				pd.load();
				
				providedDistances.put( name, pd );
				
				System.out.println("Done.");
				
			}				
			
		}		
		
	}

}
