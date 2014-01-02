#if !defined(AFX_ROBOTCMDDOC_H__A749F97E_AE41_4086_A5F0_C7148812E9C7__INCLUDED_)
#define AFX_ROBOTCMDDOC_H__A749F97E_AE41_4086_A5F0_C7148812E9C7__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RobotType.h"
// RobotCmdDoc.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CRobotCmdDoc document

class CRobotCmdDoc : public CDocument
{
protected:
	CRobotCmdDoc();           // protected constructor used by dynamic creation
	DECLARE_DYNCREATE(CRobotCmdDoc)

// Attributes
public:

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CRobotCmdDoc)
	public:
	virtual void Serialize(CArchive& ar);   // overridden for document i/o
	protected:
	virtual BOOL OnNewDocument();
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CRobotCmdDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

	// Generated message map functions
protected:
	//{{AFX_MSG(CRobotCmdDoc)
		// NOTE - the ClassWizard will add and remove member functions here.
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_ROBOTCMDDOC_H__A749F97E_AE41_4086_A5F0_C7148812E9C7__INCLUDED_)
