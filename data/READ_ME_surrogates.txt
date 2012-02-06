the current system supposes:
each sample has a txt file containing the vector of surrogate values associated with the dataset (try to name them so that they can be paired with the right sample...)

a priori, I see two different cases for surrogate variables: either they are continuous (type 1) or categorical (type 0) data. The application needs to know about it. That's the role of the hand_surrogates_types.txt file ...

in the current case, I imagined the following:
1st variable: categorical variable representing gender (0=woman, 1=man)
2nd variable: continuous  variable representing age
3rd variable: continuous  variable representing weight
4th variable: continuous  variable representing height

Finally, when conditioning a statistical model, the user should tell against which variables he wants to condition, and what values he wants to use.
the file conditioning_information.txt has two columns:
the first one indicates whether the variable is used for conditioning (0=no, 1=yes)
the second indicates values for the condition (the value is ignored if the variable is not used for conditioning)
in the current example, we want a statistical model that is specialized for women with height 162 cm.
