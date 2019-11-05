package br.ufsc.trajectoryclassification.model.vo;

public class Range implements IRange {

	private int start;

	private int end;
	
	private int moveletSize;
	
	public Range(int start, int end, int moveletSize) {
		super();
		this.start = start;
		this.end = end;
		this.moveletSize = moveletSize;
	}
	
	public int getStart() {
		return start;
	}
	
	public int getEnd() {
		return end;
	}
	
	public int getMoveletSize() {
		return moveletSize;
	}
	
	public void setStart(int start) {
		this.start = start;
	}
	
	public void setEnd(int end) {
		this.end = end;
	}
	
	public void setMoveletSize(int moveletSize) {
		this.moveletSize = moveletSize;
	}
	
	public String toString() {
	
		String phrase = Integer.toString(this.start);
		phrase = phrase + " " + Integer.toString(this.end);
		phrase = phrase + " " + Integer.toString(this.moveletSize);	
		return phrase;
		
	}
	
	
}
