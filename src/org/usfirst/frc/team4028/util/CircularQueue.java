package org.usfirst.frc.team4028.util;

import java.util.ArrayList;

// This generic class implements a circular queue
// created in 2017 to support cycling between "n" cameras
public class CircularQueue<E> extends ArrayList<E> {
	private static final long serialVersionUID = 1L;
	private int _index = 0;
	
	public E getNext () {
		return get (_index + 1);
	}
	
	public E get (int index) {
		if(index == -1) {
			index = size()-1;
		}
		else if(index == size()) {
			index = 0;
		}
		
		_index = index;
		return super.get(index);
	}
}