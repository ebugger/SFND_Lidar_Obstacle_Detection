# Radar Target Generation and Detection

## Getting Started

this will include below information for this final project:

1.Implementation steps for the 2D CFAR process.

2.Selection of Training, Guard cells and offset.

3.Steps taken to suppress the non-thresholded cells at the edges.

- Implementation steps for the 2D CFAR process.

  1.as the Training/Guard/CUT cells slide in the cell Matrix, first figure out the CUT cell moving position/dimension scope, this scope
    will not include the edeg where the CUT could not located in.
    
  2.in one of the above scope, figure out the Grid size which including Training,Guard and CUT cells. 
  
  3.in the grid above, filter the Traing cell and computer the average noise threshold from them.
  
  4.compare the average noise threshold with the CUT cell and update the CUT cell implace.
  
 
- Selection of Training, Guard cells and offset.
```
%number of Training Cells in both the dimensions

Tr = 10;
Td = 5;

%number of Guard Cells in both dimensions 

Gr = 4;
Gd = 4;

% offset the threshold by SNR value in dB

offset = 1.2;
```

- Steps taken to suppress the non-thresholded cells at the edges.

  Traversal the whole cell Matrix and set the element value to 0 whose value is neither 0 nor 1.
 



## Authors

* **Younger Liu** - *Initial work* - [Yang Liu](https://github.com/ebugger)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc
