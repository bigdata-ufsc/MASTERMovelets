package br.ufsc.trajectoryclassification.model.bo.pivots;

import java.util.List;
import java.util.Objects;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;


public class GetOutsidePivots {

	public List<List<String>> outside_Pivots;
	
	private List<String> getRecordFromLine(String line) {
	    List<String> values = new ArrayList<String>();
	    try (Scanner rowScanner = new Scanner(line)) {
	        rowScanner.useDelimiter(",");
	        while (rowScanner.hasNext()) {
	            values.add(rowScanner.next());
	        }
	    }
	    return values;
	}
	
	public List<List<String>> readCSV(String pivots_path){
		
		List<List<String>> records = new ArrayList<>();
		
		Scanner scanner;
		try {
			scanner = new Scanner(new File(pivots_path));	
			while (scanner.hasNextLine()) {
		        records.add(getRecordFromLine(scanner.nextLine()));
		    }
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		return(records);
		
	}
	
	public List<String> getTFIDFHeader(List<List<String>> csv_lines){
		
		List<String> poi_line = csv_lines.get(0);
		
		return(poi_line);
		
	}
	
	public List<String> getClassLine(Double trajectory_label, List<List<String>> csv_lines){
		
		List<String> label_line = csv_lines.get(0);
		
		List<List<String>> aux_csv_lines = csv_lines;

		for (int i = 1; i <= csv_lines.size(); i++) {
			
			String aux_csv_label = csv_lines.get(i).get(0);
			
			Double aux_csv_label_int = Double.parseDouble(aux_csv_label);
			
			if(Objects.equals(aux_csv_label_int, trajectory_label) ) return csv_lines.get(i);
			
		}
		return label_line;
	}
	
}
