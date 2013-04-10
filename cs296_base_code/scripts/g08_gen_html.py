from re import sub

tex_location="../doc/report_cs296_08.tex"
tex=open(tex_location)
report=tex.readlines()

subsection="\subsection{Graph"
section="\section"

#returns the subsection for graph-'n' as it is 
#from the .tex file
def parse_subsection(n):
	j=0
	i=1
	while(j<len(report)):

		#start line condition
		if subsection in report[j]:
			if i==n:
				j+=1
				found=[]
				found.append(report[j-1][:-1])

				#stop line condition
				while( j<len(report) and (section not in report[j]) and (subsection not in report[j])):
					found.append(report[j][:-1])
					j+=1
				return found
			else:
				i+=1
		j+=1
	return ""

#converts the latex text to html text
def convert_subsection(subsec):

	#'/subsection'=>'<h2>'
	subsec[0] = subsec[0].replace("\subsection{","<h2>")
	subsec[0] = subsec[0].replace("}","</h2>")

	#image locations are saved
	img  = subsec[1][subsec[1].index('{')+1 : subsec[1].index('}')]
	temp = subsec[1][ subsec[1].index('}')+1 :]
	img2 = temp[temp.index('{')+1 : temp.index('}')]

	#'/includegraphicx'=>'<img>'
	#Text inside <p>
	subsec[1] = "<img src='" + img +".png'>" + "\n" 
	subsec[1] += "<img src='" + img2 +".png'>" + "\n<p>" 
	i=2
	while(i < len(subsec)):
		#'\\'=>'<br>'
		#note that while saving the text, we get \\ for \
		#as \ is escaped by \
		subsec[i] = subsec[i].replace("\\\\","<br>")
		i+=1
	i=0
	temp=""
	subsec.append("</p>")
	while(i < len(subsec)):
		temp+=subsec[i]+'\n'
		i+=1
	return temp

html_body="<h1>\nGraph Analysis</h1>\n"
for i in range(1,7):
	html_body+=convert_subsection(parse_subsection(i))

#head part
html_head = '''<head>\n<title>\nGroup-08 Project Report\n</title>\n<link rel="stylesheet" type="text/css" href="g08_report.css" />\n</head>\n'''

html_code = "<html>\n" + html_head + "<body>\n"+ html_body + "</body>\n</html>"

html=open("../doc/g08_report.html","w")
html.write(html_code)
html.close()
