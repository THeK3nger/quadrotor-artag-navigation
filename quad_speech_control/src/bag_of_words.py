class bag_of_words:
	def __init__(self,filename, k):
		self.f = open(filename, 'r')
		self.labels = {}
		self.number_of_sentences=0.0
		self.dictionary={}
		self.count_labels()
		self.build_dictionary()
		self.ls=k

	# in the dictionary self.labels for each label in the corpus there is a list of two element
	# the first is number of sentences having that label and the second is the total number of
	# words labelled
	def count_labels(self):
		self.f.seek(0,0)
		for s in self.f:
			sentence = s.split()
			self.number_of_sentences+=1.0
			lab = sentence.pop()
			if (lab in self.labels):
				self.labels[lab]=[(x + y) for x, y in zip(self.labels[lab],[1,len(sentence)])]
			else:
				self.labels[lab]=[1.0,len(sentence)]

	# in the dictionary for each word in the corpus is stored a dictonary with the labels and the 
	# number of time the word was labeled
	def build_dictionary(self):
		self.f.seek(0, 0)
		for s in self.f:
			sentence = s.split()
			lab = sentence.pop()
			for w in sentence:
				if (w in self.dictionary):
					self.dictionary[w][lab]+=1.0
				else:
					self.dictionary[w]={}
					for l in self.labels:
						self.dictionary[w][l]=0.0
					self.dictionary[w][lab]+=1.0

	def p_of_word_given_label(self,w,l):
		if(w in self.dictionary):
			p= (self.dictionary[w][l]+self.ls)/(self.labels[l][1]+self.ls*len(self.dictionary))
		else:
			#laplacian smoothing??
			p= (1.0/len(self.labels))#/self.labels[l][1]		
		return p
	
	def p_of_label(self, l):
		p=(self.labels[l][0] + self.ls)/(self.number_of_sentences + self.ls*len(self.labels))
		return p

	def p_of_label_given_word(self,w,l):
		num = self.p_of_word_given_label(w,l)*self.p_of_label(l)
		den = 0.0
		for lab in self.labels:
			den = den +  self.p_of_word_given_label(w,lab)*self.p_of_label(lab)
		p=num/den
		return p

	def p_of_sentence_given_label(self,s,l):
		sentence=s.split()
		p=1.0
		for w in sentence:
			p=p*self.p_of_word_given_label(w,l)
		return p
	
	def p_of_label_given_sentence(self,l,s):
		num=self.p_of_sentence_given_label(s,l)*self.p_of_label(l)
		den=0
		for lab in  self.labels:
			#print den
			den = den + self.p_of_sentence_given_label(s,lab)*self.p_of_label(lab)
		p=num/den
		return 	p

	def infer_label(self, sentence):
		res = {}
		for lab in  self.labels:
			res[lab]=self.p_of_label_given_sentence(lab,sentence)
		return res

#asd = bag_of_words('corpus.txt', 1)
#print asd.infer_label('today is secret')
