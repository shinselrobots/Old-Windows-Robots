// PathDoc.h : CPathDoc class
// Data store for Paths; Segments, and Waypoints
/////////////////////////////////////////////////////////////////////////////
#if !defined(AFX_PATHDOC_H__8BC8A598_FAD4_4137_A17A_D45F76FE96C0__INCLUDED_)
#define AFX_PATHDOC_H__8BC8A598_FAD4_4137_A17A_D45F76FE96C0__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#include "RobotType.h"
#include "PathStruct.h"

#define MAX_MAP_SIZE 0xFFFFF	// Tenth Inches.  One mile in tenth inches = 0x9AB00



/////////////////////////////////////////////////////////////////////////////
// CPathDoc document

class CPathDoc : public CDocument
{
protected:
	CPathDoc();           // protected constructor used by dynamic creation
	DECLARE_DYNCREATE(CPathDoc)

// Attributes
public:
	CSegmentStructList m_SegmentList;
	CWaypointStructList m_WaypointList;

// Operations
public:

// Overrides
	virtual void DeleteContents();
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CPathDoc)
	public:
	virtual void Serialize(CArchive& ar);   // overridden for document i/o
	virtual BOOL OnOpenDocument(LPCTSTR lpszPathName);
	virtual BOOL OnSaveDocument(LPCTSTR lpszPathName);
	protected:
	virtual BOOL OnNewDocument();
	virtual BOOL SaveModified();
	//}}AFX_VIRTUAL

// Implementation
public:
	BOOL m_CustomFileOpen;
	virtual ~CPathDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

	// Generated message map functions
protected:
	//{{AFX_MSG(CPathDoc)
		// NOTE - the ClassWizard will add and remove member functions here.
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_PATHDOC_H__8BC8A598_FAD4_4137_A17A_D45F76FE96C0__INCLUDED_)
